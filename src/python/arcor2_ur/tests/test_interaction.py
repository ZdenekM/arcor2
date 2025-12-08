import math
import time

import pytest

from arcor2.clients import scene_service
from arcor2.data.common import Orientation, Pose, Position
from arcor2.data.object_type import Box
from arcor2.exceptions import Arcor2Exception
from arcor2_ur.object_types.ur5e import Ur5e, UrSettings
from arcor2_ur.tests.conftest import Urls


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

    orig_z = pos.position.z
    pos.position.z -= 0.05
    ot.move_to_pose("", pos, 0.5)
    pos_after = ot.get_end_effector_pose("")
    assert orig_z - pos_after.position.z > 0.045

    locked_target = Position(pos_after.position.x, pos_after.position.y, pos_after.position.z + 0.02)
    orientation_before = pos_after.orientation
    ot.move_to_position(locked_target, 50)
    locked_pose = ot.get_end_effector_pose("")
    delta_orientation = orientation_before.as_quaternion().inverse() * locked_pose.orientation.as_quaternion()
    clamped_w = max(-1.0, min(1.0, float(delta_orientation.w)))
    assert 2.0 * math.acos(clamped_w) < 0.02
    assert locked_pose.position.z - pos_after.position.z > 0.015

    joints = ot.robot_joints()
    joints[0].value -= 0.05
    ot.move_to_joints(joints, 0.5)
    moved_joints = ot.robot_joints()
    assert abs(moved_joints[0].value - joints[0].value) < 0.01
    for original, moved in zip(joints[1:], moved_joints[1:]):
        assert abs(original.value - moved.value) < 0.005

    ot.suck()
    ot.release()

    ot.move_to_pose("", pos, 0.5)

    assert not ot.get_hand_teaching_mode()

    ot.set_hand_teaching_mode(True)
    assert ot.get_hand_teaching_mode()
    time.sleep(0.5)
    assert ot.get_hand_teaching_mode()

    ot.set_hand_teaching_mode(False)
    assert not ot.get_hand_teaching_mode()

    ot.cleanup()
