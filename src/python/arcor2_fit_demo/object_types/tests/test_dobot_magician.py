import importlib.resources as pkg_resources
import os

from arcor2.object_types.utils import check_object_type
from arcor2.urdf import urdf_from_path
from arcor2_fit_demo.object_types.dobot_magician import DobotMagician


def test_dobot_magician() -> None:
    check_object_type(DobotMagician)
    assert not DobotMagician.abstract()


def test_urdf() -> None:

    with pkg_resources.path("arcor2_fit_demo", "data") as p:
        urdf_from_path(os.path.join(str(p), "dobot-magician"))
