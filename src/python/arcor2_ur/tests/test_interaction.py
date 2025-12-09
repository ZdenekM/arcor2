import time

import pytest

from arcor2.clients import scene_service
from arcor2.data.common import Orientation, Pose, Position
from arcor2.data.object_type import Box
from arcor2.exceptions import Arcor2Exception
from arcor2_ur.object_types.ur5e import Ur5e, UrSettings
from arcor2_ur.tests.conftest import Urls


def _orientation_close(a: Orientation, b: Orientation, tol: float = 2e-2) -> bool:
    return abs(a.x - b.x) <= tol and abs(a.y - b.y) <= tol and abs(a.z - b.z) <= tol and abs(a.w - b.w) <= tol


@pytest.mark.timeout(60)
def test_basics(start_processes: Urls) -> None:
    scene_service.URL = start_processes.robot_url
    box = Box("UniqueBoxId", 0.1, 0.1, 0.1)
    scene_service.upsert_collision(box, Pose(Position(1, 0, 0), Orientation(0, 0, 0, 1)))
    scene_service.start()
    assert scene_service.started()

    ot = Ur5e("", "", Pose(), UrSettings(start_processes.robot_url))

    assert len(ot.robot_joints()) == 6
    pos = ot.get_end_effector_pose("")
    orig_z = pos.position.z
    pos.position.z -= 0.05
    ot.move_to_pose("", pos, 0.5)
    pos_after = ot.get_end_effector_pose("")
    assert orig_z - pos_after.position.z > 0.045

    ot.suck()
    ot.release()

    assert scene_service.collision_ids() == {box.id}

    scene_service.upsert_collision(box, pos)
    pos.position.z += 0.01
    with pytest.raises(Arcor2Exception):  # attempt to move into a collision object
        ot.move_to_pose("", pos, 0.5)

    # now without collision checking
    ot.move_to_pose("", pos, 0.5, safe=False)

    pos.position.z -= 0.01
    with pytest.raises(Arcor2Exception):  # start state in collision
        ot.move_to_pose("", pos, 0.5)

    scene_service.delete_all_collisions()
    assert not scene_service.collision_ids()

    ot.move_to_pose("", pos, 0.5)

    stable_pose = ot.get_end_effector_pose("")
    target_position = Position(stable_pose.position.x, stable_pose.position.y, stable_pose.position.z - 0.03)
    ot.move_to_position(target_position, 40.0)
    after_position = ot.get_end_effector_pose("")
    assert _orientation_close(after_position.orientation, stable_pose.orientation)
    assert stable_pose.position.z - after_position.position.z > 0.025

    pick_pose = ot.get_end_effector_pose("")
    pick_pose.position.z -= 0.02
    ot.pick(pick_pose, velocity=30.0, vertical_offset=0.03, safe_approach=True, safe_pick=True)
    after_pick = ot.get_end_effector_pose("")
    assert _orientation_close(after_pick.orientation, pick_pose.orientation)
    assert after_pick.position.z == pytest.approx(pick_pose.position.z + 0.03, abs=0.01)

    place_pose = Pose(
        Position(pick_pose.position.x + 0.04, pick_pose.position.y, pick_pose.position.z - 0.01),
        pick_pose.orientation,
    )
    ot.place(place_pose, velocity=30.0, vertical_offset=0.02, safe_approach=True, safe_place=True)
    after_place = ot.get_end_effector_pose("")
    assert _orientation_close(after_place.orientation, place_pose.orientation)
    assert after_place.position.x == pytest.approx(place_pose.position.x, abs=0.01)
    assert after_place.position.z == pytest.approx(place_pose.position.z + 0.02, abs=0.015)

    assert not ot.get_hand_teaching_mode()

    ot.set_hand_teaching_mode(True)
    assert ot.get_hand_teaching_mode()
    time.sleep(0.5)
    assert ot.get_hand_teaching_mode()

    ot.set_hand_teaching_mode(False)
    assert not ot.get_hand_teaching_mode()

    ot.cleanup()
