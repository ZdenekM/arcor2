import asyncio

from arcor2_calibration_data import client as calib_client
from websockets.server import WebSocketServerProtocol as WsClient

from arcor2.exceptions import Arcor2Exception
from arcor2.helpers import run_in_executor
from arcor2.image import image_to_str
from arcor2.object_types.abstract import Camera
from arcor2_arserver import globals as glob
from arcor2_arserver.camera import get_camera_instance
from arcor2_arserver.decorators import scene_needed
from arcor2_arserver.scene import ensure_scene_started, update_scene_object_pose
from arcor2_arserver_data.rpc.camera import CalibrateCamera, CameraColorImage, CameraColorParameters


@scene_needed
async def camera_color_image_cb(req: CameraColorImage.Request, ui: WsClient) -> CameraColorImage.Response:

    assert glob.SCENE

    ensure_scene_started()
    camera = get_camera_instance(req.args.id)
    resp = CameraColorImage.Response()
    resp.data = image_to_str(camera.color_image())
    return resp


@scene_needed
async def camera_color_parameters_cb(
    req: CameraColorParameters.Request, ui: WsClient
) -> CameraColorParameters.Response:

    assert glob.SCENE

    ensure_scene_started()
    camera = get_camera_instance(req.args.id)
    resp = CameraColorParameters.Response()
    resp.data = camera.color_camera_params
    return resp


async def calibrate_camera(camera: Camera) -> None:

    assert camera.color_camera_params
    assert glob.SCENE

    img = await run_in_executor(camera.color_image)
    pose = await run_in_executor(calib_client.estimate_camera_pose, camera.color_camera_params, img)
    await update_scene_object_pose(glob.SCENE.object(camera.id), pose, camera)


@scene_needed
async def calibrate_camera_cb(req: CalibrateCamera.Request, ui: WsClient) -> None:

    assert glob.SCENE

    ensure_scene_started()
    camera = get_camera_instance(req.args.id)

    # TODO check camera features / meta if it supports getting color image
    if not camera.color_camera_params:
        raise Arcor2Exception("Camera parameters not available.")

    asyncio.ensure_future(calibrate_camera(camera))
