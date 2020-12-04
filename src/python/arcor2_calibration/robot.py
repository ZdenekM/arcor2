import tempfile
import shutil
import os
from urdfpy import URDF
import pyrender
import numpy as np
import cv2
import math
import quaternion
from arcor2 import transformations as tr
import open3d as o3d
import matplotlib.pyplot as plt
from PIL import Image
import copy
import time

from arcor2.data.common import Pose, Position, Orientation
from arcor2_fit_demo.object_types.kinect_azure import KinectAzure, Settings as KinectSettings
from arcor2_calibration_data import client as calib_client
from arcor2.image import image_to_cv2

from arcor2_fit_demo.object_types.abstract_dobot import MoveType
from arcor2_fit_demo.object_types.dobot_magician import DobotMagician, DobotSettings

# https://docs.opencv.org/4.4.0/dc/d2c/tutorial_real_time_pose.html

# TODO instead of pointcloud, use create_from_depth_image from open3d?


def pointcloud(depth, fov_y, aspect_ratio):  # https://github.com/mmatl/pyrender/issues/14

    fy = 0.5 / np.tan(fov_y * 0.5)
    fx = fy / aspect_ratio

    height = depth.shape[0]
    width = depth.shape[1]

    mask = np.where(depth > 0)

    x = mask[1]
    y = mask[0]

    normalized_x = (x.astype(np.float32) - width * 0.5) / width
    normalized_y = (y.astype(np.float32) - height * 0.5) / height

    world_x = normalized_x * depth[y, x] / fx
    world_y = normalized_y * depth[y, x] / fy
    world_z = depth[y, x]

    return np.vstack((world_x, world_y, world_z)).T.astype(np.float32)


def depth_image_to_np(depth: Image.Image) -> np.array:
    return np.array(depth, dtype=np.float32) / 1000.0


def draw_registration_result(obj, scene, initial_tr, icp_tr):

    initial_temp = copy.deepcopy(obj)
    aligned_temp = copy.deepcopy(obj)
    scene_temp = copy.deepcopy(scene)

    initial_temp.paint_uniform_color([1, 0, 0])
    aligned_temp.paint_uniform_color([0, 1, 0])
    scene_temp.paint_uniform_color([0, 0, 1])

    initial_temp.transform(initial_tr)
    aligned_temp.transform(icp_tr)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)

    o3d.visualization.draw_geometries([initial_temp, aligned_temp, scene_temp, mesh_frame])


def main() -> None:

    ka = KinectAzure("", "", Pose(), settings=KinectSettings("http://localhost:5016"))
    dobot = DobotMagician("", "", Pose(Position(-0.24, 0.05, 0.11)), settings=DobotSettings("/dev/ttyUSB0"))

    pil_camera_image = ka.color_image()
    ka.pose = calib_client.estimate_camera_pose(ka.color_camera_params, pil_camera_image)

    """
    orig_pose =dobot.get_end_effector_pose("")
    dobot.move(Pose(Position(z=0.01), orig_pose.orientation), MoveType.LINEAR)
    time.sleep(1)
    dobot.move(orig_pose, MoveType.LINEAR)
    """

    with tempfile.TemporaryDirectory() as temp_dir:

        urdf_package_dir = "src/python/arcor2_fit_demo/data/dobot-magician"

        target = os.path.join(temp_dir, "urdf")

        shutil.copytree(urdf_package_dir, target)

        for dname, dirs, files in os.walk(target):
            for fname in files:

                _, ext = os.path.splitext(fname)

                if ext not in (".urdf", ".xml"):
                    continue

                fpath = os.path.join(dname, fname)
                with open(fpath) as f:
                    s = f.read()
                s = s.replace("package://", "")
                with open(fpath, "w") as f:
                    f.write(s)

        robot = URDF.load(f'{target}/dobot_magician.xml')

        cfg = {joint.name: joint.value for joint in dobot.robot_joints()}

        # fk = robot.visual_trimesh_fk(cfg=cfg)
        fk = robot.collision_trimesh_fk(cfg=cfg)

        renderer = pyrender.OffscreenRenderer(viewport_width=pil_camera_image.width, viewport_height=pil_camera_image.height, point_size=1.0)

        dobot_tr_matrix = dobot.pose.as_transformation_matrix()

        print("Initial dobot_tr_matrix")
        print(dobot_tr_matrix)

        print("Averaging depth images...")

        depth_avg_num: int = 4  # TODO should be much more

        depth = depth_image_to_np(ka.depth_image())

        for _ in range(depth_avg_num-1):
            depth += depth_image_to_np(ka.depth_image())

        depth /= depth_avg_num

        camera_tr_matrix = ka.pose.as_transformation_matrix()
        camera_matrix = ka.color_camera_params.as_camera_matrix()

        _, fov_y, _, _, _ = cv2.calibrationMatrixValues(camera_matrix, (pil_camera_image.width, pil_camera_image.height), 0, 0)
        fov_y_rads = math.radians(fov_y)
        ar = pil_camera_image.width / pil_camera_image.height

        iterations = 1
        for iteration in range(iterations):

            print(f"Iteration {iteration+1}/{iterations}")

            res_mesh = o3d.geometry.TriangleMesh()

            for tm in fk:
                pose = fk[tm]
                mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(tm.vertices), triangles=o3d.utility.Vector3iVector(tm.faces))
                mesh.transform(np.dot(dobot_tr_matrix, pose))
                mesh.compute_triangle_normals()
                mesh.compute_vertex_normals()
                res_mesh += mesh

            sim_pcd = res_mesh.sample_points_uniformly(int(1e5))

            # TODO use create_from_depth_image
            real_pc = pointcloud(depth, fov_y_rads, ar)

            real_pcd = o3d.geometry.PointCloud()
            real_pcd.points = o3d.utility.Vector3dVector(real_pc)

            # inv = np.linalg.inv(camera_tr_matrix)
            # sim_pcd.transform(camera_tr_matrix)
            real_pcd.transform(camera_tr_matrix)

            bb = sim_pcd.get_axis_aligned_bounding_box()
            real_pcd = real_pcd.crop(bb.scale(1.25, bb.get_center()))
            # sim_pcd.hidden_point_removal(np.array(list(ka.pose.position)), 0.1)  # TODO fix this

            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
            o3d.visualization.draw_geometries([sim_pcd, real_pcd, mesh_frame])

            # print("Filtering out ground plane")
            # plane_model, inliers = real_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
            # real_pcd = real_pcd.select_by_index(inliers, invert=True)

            # print("Outlier removal...")
            # real_pcd.remove_radius_outlier(nb_points=20, radius=0.01)

            print("Estimating normals...")
            real_pcd.estimate_normals()

            threshold = 1.0
            trans_init = np.identity(4)

            # print("Initial alignment")
            # evaluation = o3d.pipelines.registration.evaluate_registration(sim_pcd, real_pcd, threshold, trans_init)
            # print(evaluation)

            print("Apply point-to-point ICP")

            loss = o3d.pipelines.registration.TukeyLoss(k=0.025)

            p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
            reg_p2p = o3d.pipelines.registration.registration_icp(sim_pcd, real_pcd,
                                                                  threshold, trans_init,
                                                                  p2l, o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))

            print(reg_p2p)
            print("Transformation is:")
            print(reg_p2p.transformation)
            draw_registration_result(sim_pcd, real_pcd, trans_init, reg_p2p.transformation)

            dobot_tr_matrix = np.dot(reg_p2p.transformation, dobot_tr_matrix)

            print("dobot_tr_matrix")
            print(dobot_tr_matrix)

        print("Final pose")
        pose = Pose.from_transformation_matrix(dobot_tr_matrix)
        print(pose)

        dobot.pose = pose

        dobot.move(Pose(Position(z=0.01), dobot.get_end_effector_pose("").orientation), MoveType.LINEAR)


if __name__ == "__main__":
    main()
