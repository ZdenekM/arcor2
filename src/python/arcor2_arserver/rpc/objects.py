import asyncio
from typing import Dict, List, Tuple

from dataclasses import dataclass, field

from websockets.server import WebSocketServerProtocol as WsClient

from arcor2 import helpers as hlp
from arcor2.cached import CachedProject, CachedScene
from arcor2.clients import aio_scene_service as scene_srv
from arcor2.data import events, rpc
from arcor2.data.common import Parameter, Pose, Position, SceneObject
from arcor2.data.object_type import Model3dType
from arcor2.data.scene import MeshFocusAction
from arcor2.exceptions import Arcor2Exception
from arcor2.object_types.abstract import GenericWithPose, Robot
from arcor2.source.utils import tree_to_str
from arcor2_arserver import globals as glob
from arcor2_arserver import notifications as notif
from arcor2_arserver import settings
from arcor2_arserver.clients import project_service as storage
from arcor2_arserver.helpers import ctx_write_lock, ensure_write_locked
from arcor2_arserver.object_types.data import ObjectTypeData
from arcor2_arserver.object_types.source import new_object_type
from arcor2_arserver.object_types.utils import add_ancestor_actions, object_actions, remove_object_type
from arcor2_arserver.robot import check_eef_arm, get_end_effector_pose
from arcor2_arserver.scene import ensure_scene_started, scenes, update_scene_object_pose, get_instance
from arcor2_arserver_data import events as sevts
from arcor2_arserver_data import rpc as srpc


@dataclass
class FocusedObject:

    obj_id: str
    robot: rpc.common.RobotArg
    poses: Dict[int, Pose] = field(default_factory=dict)


_focused_objects: Dict[str, FocusedObject] = {}  # key == user_name


async def focus_object_start_cb(req: srpc.o.FocusObjectStart.Request, ui: WsClient) -> None:

    scene = glob.LOCK.scene_or_exception()

    if glob.LOCK.project:
        raise Arcor2Exception("Project has to be closed first.")

    ensure_scene_started()

    obj_id = req.args.object_id
    scene_obj = scene.object(obj_id)

    if obj_id in _focused_objects:
        raise Arcor2Exception(f"Aiming already started for {scene_obj.name}.")

    obj_type = glob.OBJECT_TYPES[scene_obj.type].meta

    if not obj_type.has_pose:
        raise Arcor2Exception("Only available for objects with pose.")

    if not obj_type.object_model or obj_type.object_model.type != Model3dType.MESH:
        raise Arcor2Exception("Only available for objects with mesh model.")

    assert obj_type.object_model.mesh

    focus_points = obj_type.object_model.mesh.focus_points

    if not focus_points:
        raise Arcor2Exception("focusPoints not defined for the mesh.")

    user_name = glob.USERS.user_name(ui)

    await ensure_write_locked(req.args.object_id, user_name)
    await ensure_write_locked(req.args.robot.robot_id, user_name)

    await check_eef_arm(get_instance(req.args.robot.robot_id, Robot), req.args.robot.arm_id, req.args.robot.end_effector)

    _focused_objects[user_name] = FocusedObject(req.args.object_id, req.args.robot)
    glob.logger.info(f"Start of aiming for {obj_id}.")


async def focus_check(ui: WsClient) -> Tuple[FocusedObject, str]:

    user_name = glob.USERS.user_name(ui)

    try:
        fo = _focused_objects[user_name]
    except KeyError:
        raise Arcor2Exception("Aiming has to be started first.")

    await ensure_write_locked(fo.obj_id, user_name)
    await ensure_write_locked(fo.robot.robot_id, user_name)

    return fo, user_name


async def focus_object_cancel_cb(req: srpc.o.FocusObjectCancel.Request, ui: WsClient) -> None:

    fo, user_name = await focus_check(ui)
    _focused_objects.pop(user_name, None)
    await glob.LOCK.write_unlock([fo.obj_id, fo.robot.robot_id], user_name, True)
    if glob.LOCK.scene:
        glob.logger.info(f"Aiming for {glob.LOCK.scene.object(fo.obj_id).name} cancelled by {user_name}.")


async def focus_object_cb(req: srpc.o.FocusObject.Request, ui: WsClient) -> srpc.o.FocusObject.Response:

    scene = glob.LOCK.scene_or_exception()
    fo, user_name = await focus_check(ui)

    pt_idx = req.args.point_idx
    obj_type = glob.OBJECT_TYPES[scene.object(fo.obj_id).type].meta

    assert obj_type.has_pose
    assert obj_type.object_model
    assert obj_type.object_model.mesh

    focus_points = obj_type.object_model.mesh.focus_points

    assert focus_points

    if pt_idx < 0 or pt_idx > len(focus_points) - 1:
        raise Arcor2Exception("Index out of range.")

    robot_id, end_effector, arm_id = fo.robot.as_tuple()

    fo.poses[pt_idx] = await get_end_effector_pose(get_instance(robot_id, Robot), end_effector, arm_id)

    r = srpc.o.FocusObject.Response()
    r.data = r.Data(finished_indexes=list(fo.poses.keys()))
    return r


async def focus_object_done_cb(req: srpc.o.FocusObjectDone.Request, ui: WsClient) -> None:

    scene = glob.LOCK.scene_or_exception()
    fo, user_name = await focus_check(ui)

    obj_type = glob.OBJECT_TYPES[scene.object(fo.obj_id).type].meta
    assert obj_type.object_model
    assert obj_type.object_model.mesh

    focus_points = obj_type.object_model.mesh.focus_points

    assert focus_points

    if len(fo.poses) < len(focus_points):
        raise Arcor2Exception(f"Only {len(fo.poses)} points were done out of {len(focus_points)}.")

    obj = scene.object(fo.obj_id)
    assert obj.pose

    obj_inst = get_instance(fo.obj_id, GenericWithPose)

    fp: List[Position] = []
    rp: List[Position] = []

    for idx, pose in fo.poses.items():

        fp.append(focus_points[idx].position)
        rp.append(pose.position)

    mfa = MeshFocusAction(fp, rp)

    glob.logger.debug(f"Attempt to aim object {obj_inst.name}, data: {mfa}")

    try:
        new_pose = await scene_srv.focus(mfa)
    except scene_srv.SceneServiceException as e:
        glob.logger.error(f"Aiming failed with: {e}, mfa: {mfa}.")
        raise Arcor2Exception(f"Aiming failed. {str(e)}") from e

    glob.logger.info(f"Done aiming for {obj_inst.name}.")

    await glob.LOCK.write_unlock(fo.robot.robot_id, user_name, True)
    asyncio.create_task(update_scene_object_pose(scene, obj, new_pose, obj_inst, user_name))
    return None


async def new_object_type_cb(req: srpc.o.NewObjectType.Request, ui: WsClient) -> None:

    async with ctx_write_lock(glob.LOCK.SpecialValues.ADDING_OBJECT, ui):
        meta = req.args

        if meta.type in glob.OBJECT_TYPES:
            raise Arcor2Exception("Object type already exists.")

        hlp.is_valid_type(meta.type)

        if meta.base not in glob.OBJECT_TYPES:
            raise Arcor2Exception(
                f"Unknown base object type '{meta.base}', " f"known types are: {', '.join(glob.OBJECT_TYPES.keys())}."
            )

        base = glob.OBJECT_TYPES[meta.base]

        if base.meta.disabled:
            raise Arcor2Exception("Base object is disabled.")

        assert base.type_def is not None

        if issubclass(base.type_def, Robot):
            raise Arcor2Exception("Can't subclass Robot.")

        meta.has_pose = issubclass(base.type_def, GenericWithPose)

        if not meta.has_pose and meta.object_model:
            raise Arcor2Exception("Object without pose can't have collision model.")

        if req.dry_run:
            return None

        obj = meta.to_object_type()
        ast = new_object_type(glob.OBJECT_TYPES[meta.base].meta, meta)
        obj.source = tree_to_str(ast)

        if meta.object_model:

            if meta.object_model.type == Model3dType.MESH:

                # TODO check whether mesh id exists - if so, then use existing mesh, if not, upload a new one
                # ...get whole mesh (focus_points) based on mesh id
                assert meta.object_model.mesh
                try:
                    meta.object_model.mesh = await storage.get_mesh(meta.object_model.mesh.id)
                except storage.ProjectServiceException as e:
                    glob.logger.error(e)
                    raise Arcor2Exception(f"Mesh ID {meta.object_model.mesh.id} does not exist.")

            else:

                meta.object_model.model().id = meta.type
                await storage.put_model(meta.object_model.model())

        type_def = await hlp.run_in_executor(
            hlp.save_and_import_type_def,
            obj.source,
            obj.id,
            base.type_def,
            settings.OBJECT_TYPE_PATH,
            settings.OBJECT_TYPE_MODULE,
        )
        assert issubclass(type_def, base.type_def)
        actions = object_actions(type_def, ast)

        meta.modified = await storage.update_object_type(obj)

        glob.OBJECT_TYPES[meta.type] = ObjectTypeData(meta, type_def, actions, ast)
        add_ancestor_actions(meta.type, glob.OBJECT_TYPES)

        evt = sevts.o.ChangedObjectTypes([meta])
        evt.change_type = events.Event.Type.ADD
        asyncio.ensure_future(notif.broadcast_event(evt))
        return None


async def get_object_actions_cb(req: srpc.o.GetActions.Request, ui: WsClient) -> srpc.o.GetActions.Response:

    try:
        return srpc.o.GetActions.Response(data=list(glob.OBJECT_TYPES[req.args.type].actions.values()))
    except KeyError:
        raise Arcor2Exception(f"Unknown object type: '{req.args.type}'.")


async def get_object_types_cb(req: srpc.o.GetObjectTypes.Request, ui: WsClient) -> srpc.o.GetObjectTypes.Response:
    return srpc.o.GetObjectTypes.Response(data=[obj.meta for obj in glob.OBJECT_TYPES.values()])


def check_scene_for_object_type(scene: CachedScene, object_type: str) -> None:

    for _ in scene.objects_of_type(object_type):
        raise Arcor2Exception(f"Object type used in scene {scene.name}.")


async def delete_object_type_cb(req: srpc.o.DeleteObjectType.Request, ui: WsClient) -> None:

    async with glob.LOCK.get_lock(req.dry_run):

        try:
            obj_type = glob.OBJECT_TYPES[req.args.id]
        except KeyError:
            raise Arcor2Exception("Unknown object type.")

        if obj_type.meta.built_in:
            raise Arcor2Exception("Can't delete built-in type.")

        for obj in glob.OBJECT_TYPES.values():
            if obj.meta.base == req.args.id:
                raise Arcor2Exception(f"Object type is base of '{obj.meta.type}'.")

        async for scene in scenes():
            check_scene_for_object_type(scene, req.args.id)

        if glob.LOCK.scene:
            check_scene_for_object_type(glob.LOCK.scene, req.args.id)

        if req.dry_run:
            return

        await storage.delete_object_type(req.args.id)

        # do not care so much if delete_model fails
        if obj_type.meta.object_model:
            try:
                await storage.delete_model(obj_type.meta.object_model.model().id)
            except storage.ProjectServiceException as e:
                glob.logger.error(str(e))

        del glob.OBJECT_TYPES[req.args.id]
        await remove_object_type(req.args.id)

        evt = sevts.o.ChangedObjectTypes([obj_type.meta])
        evt.change_type = events.Event.Type.REMOVE
        asyncio.ensure_future(notif.broadcast_event(evt))


def check_override(
    scene: CachedScene, project: CachedProject, obj_id: str, override: Parameter, add_new_one: bool = False
) -> SceneObject:

    obj = scene.object(obj_id)

    for par in glob.OBJECT_TYPES[obj.type].meta.settings:
        if par.name == override.name:
            if par.type != override.type:
                raise Arcor2Exception("Override can't change parameter type.")
            break
    else:
        raise Arcor2Exception("Unknown parameter name.")

    if add_new_one:
        try:
            for existing_override in project.overrides[obj.id]:
                if override.name == existing_override.name:
                    raise Arcor2Exception("Override already exists.")
        except KeyError:
            pass
    else:
        if obj.id not in project.overrides:
            raise Arcor2Exception("There are no overrides for the object.")

        for override in project.overrides[obj.id]:
            if override.name == override.name:
                break
        else:
            raise Arcor2Exception("Override not found.")

    return obj


async def add_override_cb(req: srpc.o.AddOverride.Request, ui: WsClient) -> None:

    scene = glob.LOCK.scene_or_exception()
    project = glob.LOCK.project_or_exception()

    obj = check_override(scene, project, req.args.id, req.args.override, add_new_one=True)

    await ensure_write_locked(req.args.id, ui)

    if req.dry_run:
        return

    if obj.id not in project.overrides:
        project.overrides[obj.id] = []

    project.overrides[obj.id].append(req.args.override)
    project.update_modified()

    evt = sevts.o.OverrideUpdated(req.args.override)
    evt.change_type = events.Event.Type.ADD
    evt.parent_id = req.args.id
    asyncio.ensure_future(notif.broadcast_event(evt))


async def update_override_cb(req: srpc.o.UpdateOverride.Request, ui: WsClient) -> None:

    scene = glob.LOCK.scene_or_exception()
    project = glob.LOCK.project_or_exception()

    obj = check_override(scene, project, req.args.id, req.args.override)

    await ensure_write_locked(req.args.id, ui)

    if req.dry_run:
        return

    for override in project.overrides[obj.id]:
        if override.name == override.name:
            override.value = req.args.override.value
    project.update_modified()

    evt = sevts.o.OverrideUpdated(req.args.override)
    evt.change_type = events.Event.Type.UPDATE
    evt.parent_id = req.args.id
    asyncio.ensure_future(notif.broadcast_event(evt))


async def delete_override_cb(req: srpc.o.DeleteOverride.Request, ui: WsClient) -> None:

    scene = glob.LOCK.scene_or_exception()
    project = glob.LOCK.project_or_exception()

    obj = check_override(scene, project, req.args.id, req.args.override)

    await ensure_write_locked(req.args.id, ui)

    if req.dry_run:
        return

    project.overrides[obj.id] = [ov for ov in project.overrides[obj.id] if ov.name != req.args.override.name]

    if not project.overrides[obj.id]:
        del project.overrides[obj.id]

    project.update_modified()

    evt = sevts.o.OverrideUpdated(req.args.override)
    evt.change_type = events.Event.Type.REMOVE
    evt.parent_id = req.args.id
    asyncio.ensure_future(notif.broadcast_event(evt))
