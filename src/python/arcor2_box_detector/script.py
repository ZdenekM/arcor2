from arcor2.data.common import Pose, Position
from arcor2_fit_demo.object_types.kinect_azure import KinectAzure, Settings
from arcor2_calibration_data import client as calib_client
import open3d as o3d
from PIL import Image
import numpy as np
import copy


# TODO duplication with calibration
def depth_image_to_np(depth: Image.Image) -> np.array:
    return np.array(depth, dtype=np.float32) / 1000.0


def draw_registration_result(obj, scene, initial_tr, icp_tr) -> None:

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

    ka = KinectAzure("", "", Pose(), settings=Settings("http://localhost:5016"))
    ka.pose = calib_client.estimate_camera_pose(ka.color_camera_params, ka.color_image())

    depth_image = ka.depth_image()
    depth = depth_image_to_np(depth_image)

    camera_tr_matrix = ka.pose.as_transformation_matrix()

    # TODO undistort depth image

    colors_np = np.array(ka.color_image())

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(colors_np), o3d.geometry.Image(depth), convert_rgb_to_intensity=False, depth_scale=1)

    # TODO duplication with calibration
    real_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
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

    real_pcd.transform(camera_tr_matrix)
    real_pcd = real_pcd.voxel_down_sample(voxel_size=0.005)
    real_pcd.estimate_normals()

    green_candidates = []

    for point, color in zip(np.asarray(real_pcd.points), np.asarray(real_pcd.colors)):

        # TODO work rather in  HSV
        if color[0] < 44.0/255 and color[1] > 164.0/255 and color[2] < 95.0/255:
            # print(f"found candidate point {point} with color {color}")
            if not green_candidates:
                # print("Adding first")
                green_candidates.append(point)
            else:
                for idx in range(len(green_candidates)):
                    green_candidate = green_candidates[idx]
                    if np.linalg.norm(point-green_candidate) < 0.02:
                        # print("Averaging")
                        green_candidates[idx] = (green_candidate + point)/2.0
                        break
                else:
                    # print("Adding new one")
                    green_candidates.append(point)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    # TODO transformovat tak aby byl stred ve stredu
    o3d.visualization.draw_geometries([real_pcd, mesh_frame])

    box_mesh = o3d.geometry.TriangleMesh.create_box(0.025, 0.025, 0.025)
    box_pcd = box_mesh.sample_points_uniformly(1000)

    threshold = 1.0
    pt = green_candidates[0]
    trans_init = Pose(Position(pt[0], pt[1], pt[2])).as_transformation_matrix()
    print(trans_init)

    loss = o3d.pipelines.registration.TukeyLoss(k=0.25)

    # TODO pouzit registration_colored_icp
    p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        box_pcd,
        real_pcd,
        threshold,
        trans_init,
        p2l,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
    )

    print(reg_p2p.transformation)

    draw_registration_result(box_pcd, real_pcd, trans_init, reg_p2p.transformation)


if __name__ == "__main__":
    main()