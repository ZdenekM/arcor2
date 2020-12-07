import tempfile
import shutil
import os
from urdfpy import URDF
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
from PIL import Image
import copy
import time

from arcor2.data.common import Pose, Position, Orientation
from arcor2_fit_demo.object_types.kinect_azure import KinectAzure, Settings as KinectSettings
from arcor2_calibration_data import client as calib_client

from arcor2_fit_demo.object_types.abstract_dobot import MoveType
from arcor2_fit_demo.object_types.dobot_magician import DobotMagician, DobotSettings


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

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    o3d.visualization.draw_geometries([initial_temp, aligned_temp, scene_temp, mesh_frame])


def main() -> None:

    ka = KinectAzure("", "", Pose(), settings=KinectSettings("http://localhost:5016"))
    dobot = DobotMagician("", "", Pose(Position(-0.14-0.06, -0.22-0.04, 0.11-0.03), Orientation(0, 0, 0.707, 0.707)), settings=DobotSettings("/dev/ttyUSB0"))

    pil_camera_image = ka.color_image()
    ka.pose = calib_client.estimate_camera_pose(ka.color_camera_params, pil_camera_image)

    calib_pose = Pose(Position(x=-0.07, y=0.03, z=0.075), Orientation(1.0, 0, 0, 0))
    dobot.move(calib_pose, MoveType.LINEAR)

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

        fk = robot.visual_trimesh_fk(cfg=cfg)

        dobot_tr_matrix = dobot.pose.as_transformation_matrix()

        print("Initial dobot_tr_matrix")
        print(dobot_tr_matrix)

        print("Averaging depth images...")

        depth_avg_num: int = 128  # TODO should be much more (but it is too slow)

        depth = depth_image_to_np(ka.depth_image())

        for _ in range(depth_avg_num-1):
            depth += depth_image_to_np(ka.depth_image())

        depth /= depth_avg_num

        camera_tr_matrix = ka.pose.as_transformation_matrix()
        camera_matrix = ka.color_camera_params.as_camera_matrix()

        wh = (pil_camera_image.width, pil_camera_image.height)

        dist = np.array(ka.color_camera_params.dist_coefs)
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, wh, 1, wh)
        depth = cv2.undistort(depth, camera_matrix, dist, None, newcameramtx)

        res_mesh = o3d.geometry.TriangleMesh()

        for tm in fk:
            pose = fk[tm]
            mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(tm.vertices), triangles=o3d.utility.Vector3iVector(tm.faces))
            mesh.transform(np.dot(dobot_tr_matrix, pose))
            mesh.compute_triangle_normals()
            mesh.compute_vertex_normals()
            res_mesh += mesh

        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        # o3d.visualization.draw_geometries([res_mesh, mesh_frame])

        sim_pcd = res_mesh.sample_points_uniformly(int(1e5))

        real_pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth), o3d.camera.PinholeCameraIntrinsic(pil_camera_image.width, pil_camera_image.height, ka.color_camera_params.fx, ka.color_camera_params.fy, ka.color_camera_params.cx, ka.color_camera_params.cy))

        real_pcd.transform(camera_tr_matrix)

        bb = sim_pcd.get_axis_aligned_bounding_box()
        real_pcd = real_pcd.crop(bb.scale(1.25, bb.get_center()))

        print("Outlier removal...")
        real_pcd = real_pcd.remove_radius_outlier(nb_points=50, radius=0.01)[0]

        sim_pcd = sim_pcd.select_by_index(sim_pcd.hidden_point_removal(np.array(list(ka.pose.position)), 500)[1])

        print("Estimating normals...")
        real_pcd.estimate_normals()

        o3d.visualization.draw_geometries([sim_pcd, real_pcd, mesh_frame])

        threshold = 1.0
        trans_init = np.identity(4)

        print("Apply point-to-point ICP")

        loss = o3d.pipelines.registration.TukeyLoss(k=0.25)

        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        reg_p2p = o3d.pipelines.registration.registration_icp(sim_pcd, real_pcd,
                                                              threshold, trans_init,
                                                              p2l, o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000))

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
