import json
from typing import NamedTuple, Optional

import numpy as np
import pyk4a
from PIL import Image
from pyk4a import Config, ImageFormat, K4AException, PyK4A, PyK4ACapture

from arcor2.data.camera import CameraParameters
from arcor2.data.common import ActionMetadata, Pose
from arcor2.data.object_type import Models
from arcor2.exceptions import Arcor2Exception
from arcor2.object_types.abstract import Camera, Settings


class KinectAzureException(Arcor2Exception):
    pass


class ColorAndDepthImage(NamedTuple):

    color: Image.Image
    depth: Image.Image


class KinectAzure(Camera):
    def __init__(
        self,
        obj_id: str,
        name: str,
        pose: Pose,
        collision_model: Optional[Models] = None,
        settings: Optional[Settings] = None,
    ) -> None:

        super(KinectAzure, self).__init__(obj_id, name, pose, collision_model, settings)

        self._k4a = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_720P,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                synchronized_images_only=True,
                color_format=ImageFormat.COLOR_BGRA32,
            )
        )
        self._k4a.start()
        self._k4a.exposure_mode_auto = True

        c = json.loads(self._k4a.calibration_raw)

        # https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/opencv_compatibility/main.cpp
        # https://github.com/etiennedub/pyk4a/issues/69#issuecomment-698756626

        # order of parameters in JSON hopefully corresponds to order of parameters in this struct
        # https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html

        # "CALIBRATION_LensDistortionModelBrownConrady"

        img = self.color_image()

        try:
            for camera in c["CalibrationInformation"]["Cameras"]:

                params = camera["Intrinsics"]["ModelParameters"]

                # TODO
                # float cx = params->param.cx * mode_info->calibration_image_binned_resolution[0];
                # cx -= mode_info->crop_offset[0];
                # cy -= mode_info->crop_offset[1];
                cx = params[0] * img.width
                cy = params[1] * img.height

                fx = params[2] * img.width
                fy = params[3] * img.height

                k1 = params[4]
                k2 = params[5]
                k3 = params[6]
                k4 = params[7]
                k5 = params[8]
                k6 = params[9]

                p2 = params[12]
                p1 = params[13]

                cp = CameraParameters(cx, cy, fx, fy, [k1, k2, p1, p2, k3, k4, k5, k6])

                if camera["Location"] == "CALIBRATION_CameraLocationD0":
                    assert self.depth_camera_params is None
                    self.depth_camera_params = cp
                elif camera["Location"] == "CALIBRATION_CameraLocationPV0":
                    assert self.color_camera_params is None
                    self.color_camera_params = cp

        except (KeyError, IndexError) as e:
            raise KinectAzureException("Failed to parse calibration.") from e

        if self.color_camera_params is None or self.depth_camera_params is None:
            raise KinectAzureException("Failed to get camera calibration.")

    def _bgra_to_rgba(self, arr: np.ndarray) -> None:

        arr[:, :, [0, 1, 2, 3]] = arr[:, :, [2, 1, 0, 3]]

    def _capture(self) -> PyK4ACapture:

        try:
            return self._k4a.get_capture(timeout=1000)
        except K4AException as e:
            raise KinectAzureException("Failed to get capture.") from e

    def color_image(self) -> Image.Image:

        capture = self._capture()

        if not np.any(capture.color):
            raise KinectAzureException("Color image not available.")

        self._bgra_to_rgba(capture.color)
        return Image.fromarray(capture.color, mode="RGBA")

    def depth_image(self) -> Image.Image:

        capture = self._capture()

        if not np.any(capture.depth):
            raise KinectAzureException("Depth image not available.")

        return Image.fromarray(capture.depth)

    def sync_images(self) -> ColorAndDepthImage:

        capture = self._capture()

        if not np.any(capture.color) or not np.any(capture.depth):
            raise KinectAzureException("Color/depth image not available.")

        self._bgra_to_rgba(capture.color)
        return ColorAndDepthImage(Image.fromarray(capture.color), Image.fromarray(capture.transformed_depth))

    def cleanup(self) -> None:

        super(KinectAzure, self).cleanup()
        self._k4a.stop()

    color_image.__action__ = ActionMetadata(blocking=True)  # type: ignore
    depth_image.__action__ = ActionMetadata(blocking=True)  # type: ignore
