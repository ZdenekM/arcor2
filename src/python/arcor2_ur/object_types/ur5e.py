import time
from dataclasses import dataclass
from typing import cast

from dataclasses_jsonschema import JsonSchemaMixin

from arcor2 import rest
from arcor2.data.common import ActionMetadata, Joint, Pose, Position, StrEnum
from arcor2.data.robot import InverseKinematicsRequest
from arcor2.object_types.abstract import Robot, Settings


@dataclass
class UrSettings(Settings):
    url: str = "http://ur-demo-robot-api:5012"


@dataclass
class Vacuum(JsonSchemaMixin):
    a: float
    b: float

    def avg(self) -> float:
        return (self.a + self.b) / 2


class VacuumChannel(StrEnum):
    A: str = "a"
    B: str = "b"
    BOTH: str = "both"


class Ur5e(Robot):
    _ABSTRACT = False
    urdf_package_name = "ur5e.zip"

    def __init__(self, obj_id: str, name: str, pose: Pose, settings: UrSettings) -> None:
        super(Ur5e, self).__init__(obj_id, name, pose, settings)
        self._start()

    def _started(self) -> bool:
        return rest.call(rest.Method.GET, f"{self.settings.url}/state/started", return_type=bool)

    def _stop(self) -> None:
        try:  # TODO the service crashes on stop (and is auto-restarted after that), so let's tolerate errors here
            rest.call(rest.Method.PUT, f"{self.settings.url}/state/stop")
        except rest.RestException:
            pass

    def _start(self) -> None:
        if self._started():
            self._stop()

        rest.call(rest.Method.PUT, f"{self.settings.url}/state/start", body=self.pose, timeout=rest.Timeout(read=30))

    @property
    def settings(self) -> UrSettings:
        return cast(UrSettings, super(Ur5e, self).settings)

    def cleanup(self):
        self._stop()

    def get_hand_teaching_mode(self) -> bool:
        return rest.call(rest.Method.GET, f"{self.settings.url}/hand_teaching", return_type=bool)

    def set_hand_teaching_mode(self, enabled: bool) -> None:
        rest.call(rest.Method.PUT, f"{self.settings.url}/hand_teaching", params={"enabled": enabled})

    def get_end_effectors_ids(self) -> set[str]:
        return {"default"}

    def grippers(self) -> set[str]:
        return set()

    def suctions(self) -> set[str]:
        return set("default")

    def get_end_effector_pose(self, end_effector_id: str) -> Pose:
        return rest.call(rest.Method.GET, f"{self.settings.url}/eef/pose", return_type=Pose)

    def move_to_pose(
        self, end_effector_id: str, target_pose: Pose, speed: float, safe: bool = True, linear: bool = True
    ) -> None:
        self.move(target_pose, speed * 100, safe)

    def move(
        self,
        pose: Pose,
        speed: float = 50.0,
        safe: bool = True,
        payload: float = 0.0,
        *,
        an: None | str = None,
    ) -> None:
        """Moves the robot's end-effector to a specific pose.

        :param pose: Target pose.
        :param speed: Relative speed.
        :param safe: Avoid collisions.
        :param payload: Object weight.

        :return:
        """

        assert 0.0 <= speed <= 100.0
        assert 0.0 <= payload <= 5.0

        with self._move_lock:
            rest.call(
                rest.Method.PUT,
                f"{self.settings.url}/eef/pose",
                body=pose,
                params={"velocity": speed, "payload": payload, "safe": safe},
            )

    def move_to_position(
        self, position: Position, speed: float = 50.0, safe: bool = True, payload: float = 0.0, *, an: None | str = None
    ) -> None:
        """Move the end-effector to a position while keeping orientation.

        :param position: Target position.
        :param speed: Relative speed.
        :param safe: Avoid collisions.
        :param payload: Object weight.
        """

        assert 0.0 <= speed <= 100.0
        assert 0.0 <= payload <= 5.0

        with self._move_lock:
            rest.call(
                rest.Method.PUT,
                f"{self.settings.url}/eef/position",
                body=position,
                params={"velocity": speed, "payload": payload, "safe": safe},
            )

    def suck(
        self,
        vacuum: int = 60,
        channel: VacuumChannel = VacuumChannel.BOTH,
        wait_for_vacuum: int = 1,
        min_vacuum: float = 20,
        *,
        an: None | str = None,
    ) -> bool:
        """Turns on the suction.

        :param vacuum: Desired relative level of vacuum.
        :param channel: Turn on channel A, B, or both.
        :param wait_for_vacuum: How long to wait before checking vaccuum.
        :param min_vacuum: Minimal relative vacuum for success.
        :return:
        """

        assert 0 <= vacuum <= 80
        assert 0 <= wait_for_vacuum <= 60
        assert 0.0 <= min_vacuum <= 100.0

        # TODO turn on channel according to VacuumChannel
        rest.call(rest.Method.PUT, f"{self.settings.url}/suction/suck", params={"vacuum": vacuum})
        time.sleep(wait_for_vacuum)

        vac = self.vacuum()

        if (
            channel == VacuumChannel.A
            and vac.a < min_vacuum
            or channel == VacuumChannel.B
            and vac.b < min_vacuum
            or channel == VacuumChannel.BOTH
            and vac.avg() < min_vacuum
        ):
            self.release()
            return False
        return True

    def release(self, *, an: None | str = None) -> None:
        """Turns off the suction."""

        rest.call(rest.Method.PUT, f"{self.settings.url}/suction/release")

    def vacuum(self) -> Vacuum:
        """Get vacuum on both channels."""

        return rest.call(rest.Method.GET, f"{self.settings.url}/suction/vacuum", return_type=Vacuum)

    def robot_joints(self, include_gripper: bool = False) -> list[Joint]:
        return rest.call(rest.Method.GET, f"{self.settings.url}/joints", list_return_type=Joint)

    def pick(
        self,
        pick_pose: Pose,
        velocity: float = 50.0,
        vertical_offset: float = 0.05,
        safe_approach: bool = True,
        safe_pick: bool = False,
        payload: float = 0.0,
        *,
        an: None | str = None,
    ) -> None:
        """Picks an item from given pose while keeping tool orientation.

        :param pick_pose: Where to pick an object.
        :param velocity: Speed of move (percent).
        :param vertical_offset: Vertical offset for pre/post pick pose.
        :param safe_approach: Safe approach to the pre-pick position.
        :param safe_pick: Safe picking movements.
        :param payload: Object weight.
        """

        assert 0.0 <= velocity <= 100.0
        assert vertical_offset >= 0.0
        assert 0.0 <= payload <= 5.0

        approach_pose = Pose.from_dict(pick_pose.to_dict())
        approach_pose.position.z += vertical_offset

        target_position = Position(pick_pose.position.x, pick_pose.position.y, pick_pose.position.z)
        retreat_position = Position(approach_pose.position.x, approach_pose.position.y, approach_pose.position.z)

        self.move(approach_pose, velocity, safe=safe_approach, payload=payload)  # pre-pick pose
        self.move_to_position(target_position, velocity, safe=safe_pick, payload=payload)  # pick pose
        self.suck()
        self.move_to_position(retreat_position, velocity, safe=False, payload=payload)  # back to pre-pick pose

    def place(
        self,
        place_pose: Pose,
        velocity: float = 50.0,
        vertical_offset: float = 0.05,
        safe_approach: bool = True,
        safe_place: bool = False,
        payload: float = 0.0,
        *,
        an: None | str = None,
    ) -> None:
        """Places an item to a given pose while keeping tool orientation.

        :param place_pose: Where to place the object.
        :param velocity: Speed of move (percent).
        :param vertical_offset: Vertical offset for pre/post place pose.
        :param safe_approach: Safe approach to the pre-place position.
        :param safe_place: Safe placement movements.
        :param payload: Object weight.
        """

        assert 0.0 <= velocity <= 100.0
        assert vertical_offset >= 0.0
        assert 0.0 <= payload <= 5.0

        approach_pose = Pose.from_dict(place_pose.to_dict())
        approach_pose.position.z += vertical_offset

        target_position = Position(place_pose.position.x, place_pose.position.y, place_pose.position.z)
        retreat_position = Position(approach_pose.position.x, approach_pose.position.y, approach_pose.position.z)

        self.move(approach_pose, velocity, safe=safe_approach, payload=payload)  # pre-place pose
        self.move_to_position(target_position, velocity, safe=safe_place, payload=payload)  # place pose
        self.release()
        self.move_to_position(retreat_position, velocity, safe=False, payload=payload)  # back to pre-place pose

    def inverse_kinematics(
        self,
        end_effector_id: str,
        pose: Pose,
        start_joints: None | list[Joint] = None,
        avoid_collisions: bool = True,
    ) -> list[Joint]:
        """Computes inverse kinematics.

        :param end_effector_id: IK target pose end-effector
        :param pose: IK target pose
        :param start_joints: IK start joints (not supported)
        :param avoid_collisions: Return non-collision IK result if true (not supported)
        :return: Inverse kinematics
        """

        return rest.call(
            rest.Method.PUT,
            f"{self.settings.url}/ik",
            body=InverseKinematicsRequest(pose, start_joints, avoid_collisions),
            list_return_type=Joint,
        )

    move.__action__ = ActionMetadata()  # type: ignore
    move_to_position.__action__ = ActionMetadata()  # type: ignore
    pick.__action__ = ActionMetadata(composite=True)  # type: ignore
    place.__action__ = ActionMetadata(composite=True)  # type: ignore
