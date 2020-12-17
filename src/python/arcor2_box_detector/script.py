from arcor2.data.common import Pose, Position, Orientation
from arcor2_fit_demo.object_types.kinect_azure import KinectAzure, Settings
from arcor2_calibration_data import client as calib_client
import open3d as o3d
from PIL import Image
import numpy as np
import copy
import cv2
import colorsys
from decimal import Decimal
import time
from arcor2_fit_demo.object_types.dobot_magician import DobotMagician, DobotSettings, MoveType


# TODO duplication with calibration
def depth_image_to_np(depth: Image.Image) -> np.array:
    return np.array(depth, dtype=np.float32) / 1000.0


def draw_registration_result(obj, scene, initial_tr, icp_tr) -> None:

    initial_temp = copy.deepcopy(obj)
    aligned_temp = copy.deepcopy(obj)
    scene_temp = copy.deepcopy(scene)

    # initial_temp.paint_uniform_color([1, 0, 0])
    # aligned_temp.paint_uniform_color([0, 1, 0])
    # scene_temp.paint_uniform_color([0, 0, 1])

    initial_temp.transform(initial_tr)
    aligned_temp.transform(icp_tr)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

    o3d.visualization.draw_geometries([initial_temp, aligned_temp, scene_temp, mesh_frame])


def main() -> None:

    ka = KinectAzure("", "", Pose(), settings=Settings("http://localhost:5016"))
    ka.pose = calib_client.estimate_camera_pose(ka.color_camera_params, ka.color_image())

    start = time.monotonic()

    depth_image = ka.depth_image()
    depth = depth_image_to_np(depth_image)

    # TODO duplication with calibration
    wh = (depth_image.width, depth_image.height)
    camera_matrix = ka.color_camera_params.as_camera_matrix()
    dist = np.array(ka.color_camera_params.dist_coefs)
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, wh, 1, wh)
    depth = cv2.undistort(depth, camera_matrix, dist, None, newcameramtx)

    camera_tr_matrix = ka.pose.as_transformation_matrix()

    colors_np = np.array(ka.color_image())

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(colors_np), o3d.geometry.Image(depth), convert_rgb_to_intensity=False, depth_scale=1)

    # TODO duplication with calibration
    real_pcd_full = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            depth_image.width,
            depth_image.height,
            ka.color_camera_params.fx,
            ka.color_camera_params.fy,
            ka.color_camera_params.cx,
            ka.color_camera_params.cy,
        ),
    )

    real_pcd_full.transform(camera_tr_matrix)
    real_pcd = real_pcd_full.voxel_down_sample(voxel_size=0.0025)
    real_pcd_full.estimate_normals()  # this is only for visualization

    green_candidates = []

    rgb_green = np.array([14.0 / 255, 65.0 / 255, 31.0 / 255])

    dim = 0.025
    box_mesh = o3d.geometry.TriangleMesh.create_box(dim, dim, dim)
    box_pcd = box_mesh.sample_points_uniformly(1000)
    box_pcd.paint_uniform_color(rgb_green)
    box_pcd.translate(np.array([-dim / 2, -dim / 2, -dim / 2]))

    # box_volume = Decimal(dim**3)

    green = 0
    non_green = 0

    ref_hue = colorsys.rgb_to_hsv(*rgb_green)[0]

    for point, rgb_color in zip(np.asarray(real_pcd.points), np.asarray(real_pcd.colors)):

        if np.linalg.norm(point) > 0.5:  # TODO search area should be somehow configurable
            continue

        hsv_color = colorsys.rgb_to_hsv(*rgb_color)
        hue = hsv_color[0]

        dist = min(abs(ref_hue - hue), abs(ref_hue - hue + 1))

        if dist > 0.1:
            non_green += 1
            continue

        green += 1

        # print(f"found candidate point {point} with color {hue}")
        if not green_candidates:
            # print("Adding first")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array([point]).reshape(1, 3))
            green_candidates.append(pcd)
        else:
            # TODO find lowest dist?
            for green_candidate in green_candidates:
                dist = np.linalg.norm(point-green_candidate.get_center())
                # print(f"{point}, {green_candidate.get_center()}, dist: {dist}")
                if dist < dim*1.1:
                    # print(f"Appending dist {dist}")
                    points = np.asarray(green_candidate.points)
                    points = np.append(points, np.array([point]), 0)
                    vec = o3d.utility.Vector3dVector(points)
                    green_candidate.points = vec
                    break
            else:
                # print("Adding new one")
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array([point]))
                green_candidates.append(pcd)

    print(f"green: {green}, non_green: {non_green}, candidates: {len(green_candidates)}")

    for cand in green_candidates:
        if len(cand.points) < 100:
            continue
        print(f"points:  {len(cand.points)}, center: {cand.get_center()}, volume: {Decimal(cand.get_oriented_bounding_box().volume())}")

    # green_candidates = [cand for cand in green_candidates if len(np.asarray(cand.points)) > 10 and abs(Decimal(cand.get_oriented_bounding_box().volume()) - box_volume) < Decimal(0.6)*box_volume]
    # print(len(green_candidates))

    threshold = 0.1

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([real_pcd, mesh_frame, box_pcd] + green_candidates)

    most_prob_cand = max(green_candidates, key=lambda item: len(item.points))

    pt = most_prob_cand.get_center()

    init_pose = Pose(Position(pt[0], pt[1], pt[2]))

    trans_init = init_pose.as_transformation_matrix()
    print(trans_init)

    # This is implementation of following paper
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017
    """
    voxel_radius = [0.01, 0.005, 0.0025]
    max_iter = [50, 30, 14]
    current_transformation = trans_init
    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = box_pcd.voxel_down_sample(radius)
        target_down = real_pcd.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=iter))
        current_transformation = result_icp.transformation
        print(current_transformation)

    end = time.monotonic()
    print(f"time: {end-start}")

    draw_registration_result(box_pcd, real_pcd_full, trans_init, current_transformation)
    """

    dobot = DobotMagician("", "", Pose(Position(-0.1, -0.23, 0.12), Orientation(0, 0, 0.707, 0.707)), settings=DobotSettings("/dev/ttyUSB0"))
    dobot.pose = calib_client.calibrate_robot(calib_client.CalibrateRobotArgs(dobot.robot_joints(), dobot.pose, ka.pose, ka.color_camera_params, "http://localhost:10000/models/dobot-magician.zip/mesh/file"), ka.depth_image(128))

    init_pose.position.z += 0.015
    init_pose.orientation = dobot.get_end_effector_pose("").orientation

    dobot.move(init_pose, MoveType.JUMP)
    dobot.suck()
    init_pose.position.z += 0.1
    dobot.move(init_pose, MoveType.JUMP)
    dobot.release()


if __name__ == "__main__":
    main()
