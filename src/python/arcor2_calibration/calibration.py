from typing import Dict, List

import cv2
import numpy as np
import quaternion
from cv2 import aruco
from PIL import Image

from arcor2.data.common import Orientation, Pose, Position


def get_poses(
    camera_matrix: List[List[float]], dist_matrix: List[float], image: Image.Image, marker_size: float = 0.1
) -> Dict[int, Pose]:

    camera_matrix = np.array(camera_matrix)
    dist_matrix = np.array(dist_matrix)

    gray = cv2.cvtColor(np.array(image), cv2.COLOR_RGBA2GRAY)

    ret: Dict[int, Pose] = {}

    aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
    parameters = aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG  # default is none

    corners, ids, _ = aruco.detectMarkers(
        gray, aruco_dict, cameraMatrix=camera_matrix, distCoeff=dist_matrix, parameters=parameters
    )

    if np.all(ids is None):
        return ret

    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_matrix)

    """
    backtorgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    aruco.drawDetectedMarkers(backtorgb, corners)  # Draw A square around the markers
    aruco.drawAxis(backtorgb, camera_matrix, dist_matrix, rvec, tvec, 0.15)

    cv2.imshow('image', backtorgb)
    cv2.waitKey(1000)
    """

    rvec = rvec.reshape(len(ids), 3)
    tvec = tvec.reshape(len(ids), 3)

    for idx, mid in enumerate(ids):

        # convert pose of the marker wrt camera to pose of camera wrt marker
        # based on https://stackoverflow.com/a/51515560/3142796
        marker_rot_matrix, _ = cv2.Rodrigues(rvec[idx])

        assert np.allclose(np.linalg.inv(marker_rot_matrix), marker_rot_matrix.transpose())
        assert math.isclose(np.linalg.det(marker_rot_matrix), 1)

        camera_rot_matrix = marker_rot_matrix.transpose()

        camera_trans_vector = -np.matmul(camera_rot_matrix, tvec[idx].reshape(3, 1)).flatten()

        o = Orientation()
        o.set_from_quaternion(quaternion.from_rotation_matrix(camera_rot_matrix))
        ret[mid[0]] = Pose(Position(camera_trans_vector[0], camera_trans_vector[1], camera_trans_vector[2]), o)

    return ret
