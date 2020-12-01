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

from arcor2.data.common import Pose, Position, Orientation
from arcor2_fit_demo.object_types.kinect_azure import KinectAzure, Settings as KinectSettings
from arcor2_calibration_data import client as calib_client

from arcor2_fit_demo.object_types.dobot_magician import DobotMagician, DobotSettings


# https://docs.opencv.org/4.4.0/dc/d2c/tutorial_real_time_pose.html
#

# TODO use create_from_depth_image from open3d?
def pointcloud(depth, fov_y, aspect_ratio):  # https://github.com/mmatl/pyrender/issues/14

    fy = 0.5 / np.tan(fov_y * 0.5)
    fx = fy / aspect_ratio

    height = depth.shape[0]
    width = depth.shape[1]

    mask = np.where(depth > 0)

    x = mask[1]
    y = mask[0]

    normalized_x = (x.astype(np.float32) - width * 0.5) / width
    normalized_y = -(y.astype(np.float32) - height * 0.5) / height

    world_x = normalized_x * depth[y, x] / fx
    world_y = normalized_y * depth[y, x] / fy
    world_z = -depth[y, x]

    return np.vstack((world_x, world_y, world_z)).T.astype(np.float32)


def main() -> None:

    ka = KinectAzure("", "", Pose(), settings=KinectSettings("http://localhost:5016"))
    dobot = DobotMagician("", "", Pose(Position(-0.2, 0, 0.12)), settings=DobotSettings("/dev/ttyUSB0"))

    pil_camera_image = ka.color_image()
    ka.pose = calib_client.estimate_camera_pose(ka.color_camera_params, pil_camera_image)

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

        r = pyrender.OffscreenRenderer(viewport_width=pil_camera_image.width, viewport_height=pil_camera_image.height, point_size=1.0)

        scene = pyrender.Scene()
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            scene.add(mesh, pose=pose)

        camera_matrix = ka.color_camera_params.as_camera_matrix()

        _, fov_y, _, _, _ = cv2.calibrationMatrixValues(camera_matrix, (pil_camera_image.width, pil_camera_image.height), 0, 0)

        fov_y_rads = math.radians(fov_y)
        ar = pil_camera_image.width / pil_camera_image.height

        # Set up the camera -- z-axis away from the scene, x-axis right, y-axis up
        camera = pyrender.PerspectiveCamera(yfov=fov_y_rads, znear=0.05, zfar=2.0)

        camera_pose = np.dot(ka.pose.as_transformation_matrix(), Pose(orientation=Orientation(1, 0, 0, 0)).as_transformation_matrix())
        scene.add(camera, pose=camera_pose)

        # Set up the light -- a single spot light in the same spot as the camera
        light = pyrender.SpotLight(color=np.ones(3), intensity=1.0,
                                   innerConeAngle=np.pi / 16.0)
        scene.add(light, pose=camera_pose)

        print("Rendering model...")

        sim_rgb, sim_depth = r.render(scene)
        sim_gray = cv2.cvtColor(sim_rgb, cv2.COLOR_BGR2GRAY)

        sim_pc = pointcloud(sim_depth, fov_y_rads, ar)
        real_pc = pointcloud(np.array(ka.depth_image(), dtype=np.float32)/1000.0, fov_y_rads, ar)

        print("Computing normals...")

        _, real_pc_norm = cv2.ppf_match_3d.computeNormalsPC3d(real_pc, 5, False, np.array([0, 0, 0]))

        # https://github.com/opencv/opencv_contrib/blob/master/modules/surface_matching/samples/ppf_load_match.py
        detector = cv2.ppf_match_3d_PPF3DDetector(0.05)

        print('Training...')
        detector.trainModel(sim_pc)

        print('Matching...')
        results = detector.match(real_pc_norm)

        print('Performing ICP...')
        icp = cv2.ppf_match_3d_ICP(10)
        register_failure, results = icp.registerModelToScene(sim_pc, real_pc_norm, results[:2])

        if register_failure:
            raise ValueError("Failed to register model to scene.")

        print("Poses: ")
        for i, result in enumerate(results):
            print("\n-- Pose to Model Index %d: NumVotes = %d, Residual = %f\n%s\n" % (
            result.modelIndex, result.numVotes, result.residual, result.pose))

        rest_mat = results[0].pose  # pose of the object within the scene (relative to the camera)

        tvec = rest_mat[:3, 3]

        o = Orientation()
        o.set_from_quaternion(quaternion.from_rotation_matrix(rest_mat[:3, :3]))

        p = Pose(Position(tvec[0], tvec[1], tvec[2]), o)
        print("Pose relative to camera:")
        print(p)
        ap = tr.make_pose_abs(ka.pose, p)

        print("Absolute pose:")
        print(ap)

        import open3d as o3d

        tr_sim_pc = cv2.ppf_match_3d.transformPCPose(sim_pc, rest_mat)

        sim_pcd = o3d.geometry.PointCloud()
        sim_pcd.points = o3d.utility.Vector3dVector(tr_sim_pc)
        sim_pcd.paint_uniform_color([0, 1, 0])

        real_pcd = o3d.geometry.PointCloud()
        real_pcd.points = o3d.utility.Vector3dVector(real_pc)

        o3d.visualization.draw_geometries([real_pcd, sim_pcd])


if __name__ == "__main__":
    main()
