import inspect
import logging
import os
import subprocess as sp
import tempfile
from typing import Dict, Iterator, Tuple, Type

import pytest

from arcor2.clients import persistent_storage
from arcor2.data.events import Event
from arcor2.data.rpc.common import TypeArgs
from arcor2.helpers import find_free_port
from arcor2_arserver_data import events, rpc
from arcor2_arserver_data.client import ARServer, uid
from arcor2_arserver_data.robot import RobotMeta
from arcor2_execution_data import EVENTS as EXE_EVENTS
from arcor2_fit_demo.object_types.dobot_m1 import DobotM1
from arcor2_fit_demo.object_types.dobot_magician import DobotMagician

LOGGER = logging.getLogger(__name__)


_arserver_port: int = 0


def log_proc_output(out: Tuple[bytes, bytes]) -> None:

    for line in out[0].decode().splitlines():
        LOGGER.error(line)


def finish_processes(processes) -> None:

    for proc in processes:
        proc.terminate()
        proc.wait()
        log_proc_output(proc.communicate())


@pytest.fixture()
def start_processes() -> Iterator[None]:

    global _arserver_port

    _arserver_port = find_free_port()

    with tempfile.TemporaryDirectory() as tmp_dir:

        my_env = os.environ.copy()

        project_port = find_free_port()
        project_url = f"http://0.0.0.0:{project_port}"
        my_env["ARCOR2_PERSISTENT_STORAGE_URL"] = project_url
        my_env["ARCOR2_PROJECT_SERVICE_MOCK_PORT"] = str(project_port)
        persistent_storage.URL = project_url

        my_env["ARCOR2_EXECUTION_URL"] = f"ws://0.0.0.0:{find_free_port()}"
        my_env["ARCOR2_PROJECT_PATH"] = os.path.join(tmp_dir, "packages")

        my_env["ARCOR2_SERVER_PORT"] = str(_arserver_port)
        my_env["ARCOR2_DATA_PATH"] = os.path.join(tmp_dir, "data")

        processes = []

        for cmd in (
            "./src.python.arcor2_mocks.scripts/mock_project.pex",
            "./src.python.arcor2_execution.scripts/execution.pex",
        ):
            processes.append(sp.Popen(cmd, env=my_env, stdout=sp.PIPE, stderr=sp.STDOUT))

        # it may take some time for project service to come up so give it some time
        for _ in range(3):
            upload_proc = sp.Popen(
                "./src.python.arcor2_fit_demo.scripts/upload_objects.pex", env=my_env, stdout=sp.PIPE, stderr=sp.STDOUT
            )
            ret = upload_proc.communicate()
            if upload_proc.returncode == 0:
                log_proc_output(ret)
                break
        else:
            raise Exception("Failed to upload objects.")

        LOGGER.info(f"Starting ARServer listening on port  {_arserver_port}.")

        arserver_proc = sp.Popen(
            "./src.python.arcor2_arserver.scripts/arserver.pex", env=my_env, stdout=sp.PIPE, stderr=sp.STDOUT
        )

        processes.append(arserver_proc)
        assert arserver_proc.stdout is not None

        while True:
            line = arserver_proc.stdout.readline().decode().strip()
            if not line or "Server initialized." in line:  # TODO this is not ideal
                break

        if arserver_proc.poll():
            finish_processes(processes)
            raise Exception("ARServer died.")

        yield None

        finish_processes(processes)


def ars_connection_str() -> str:
    return f"ws://0.0.0.0:{_arserver_port}"


# TODO refactor this into _data packages
event_mapping: Dict[str, Type[Event]] = {evt.__name__: evt for evt in EXE_EVENTS}

modules = []

for _, mod in inspect.getmembers(events, inspect.ismodule):
    modules.append(mod)

for mod in modules:
    for _, cls in inspect.getmembers(mod, inspect.isclass):
        if issubclass(cls, Event):
            event_mapping[cls.__name__] = cls


@pytest.fixture()
def ars() -> Iterator[ARServer]:

    with ARServer(ars_connection_str(), timeout=30, event_mapping=event_mapping) as ws:
        yield ws


def test_objects(start_processes: None, ars: ARServer) -> None:

    assert isinstance(ars.get_event(), events.c.ShowMainScreen)

    res = ars.call_rpc(rpc.o.GetObjectTypes.Request(uid()), rpc.o.GetObjectTypes.Response)
    assert res.result
    assert res.data is not None

    for obj in res.data:
        assert not obj.disabled, f"ObjectType {obj.type} disabled. {obj.problem}"

        actions = ars.call_rpc(rpc.o.GetActions.Request(uid(), TypeArgs(obj.type)), rpc.o.GetActions.Response)
        assert actions.result
        assert actions.data is not None

        for act in actions.data:
            assert act.disabled == (act.problem is not None)
            assert not act.disabled, f"Action {act.name} of {obj.type} disabled. {act.problem}"


def test_robot_meta(start_processes: None, ars: ARServer) -> None:

    assert isinstance(ars.get_event(), events.c.ShowMainScreen)

    res = ars.call_rpc(rpc.r.GetRobotMeta.Request(uid()), rpc.r.GetRobotMeta.Response)
    assert res.result
    assert res.data is not None

    robots: Dict[str, RobotMeta] = {robot.type: robot for robot in res.data}

    magician = robots[DobotMagician.__name__]
    assert magician.features.move_to_pose
    assert magician.features.move_to_joints
    assert magician.features.inverse_kinematics
    assert magician.features.forward_kinematics
    assert not magician.features.stop
    assert not magician.features.hand_teaching

    m1 = robots[DobotM1.__name__]
    assert m1.features.move_to_pose
    assert m1.features.move_to_joints
    assert m1.features.hand_teaching
    assert not m1.features.inverse_kinematics
    assert not m1.features.forward_kinematics
    assert not m1.features.stop
