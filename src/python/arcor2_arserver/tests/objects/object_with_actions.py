import random

from arcor2.data.common import ActionMetadata, IntEnum
from arcor2.object_types.abstract import Generic, Settings


class MyEnum(IntEnum):
    ONE = 1
    TWO = 2
    THREE = 3


class ObjectWithActions(Generic):
    """Object with actions that even return something."""

    _ABSTRACT = False

    def __init__(self, obj_id: str, name: str, settings: None | Settings = None) -> None:
        super(ObjectWithActions, self).__init__(obj_id, name, settings)

    def _random_bool(self) -> bool:
        return random.choice([False, True])

    def str_action(self, *, an: None | str = None) -> str:
        return "Hello world."

    def bool_action(self, *, an: None | str = None) -> bool:
        return self._random_bool()

    def tuple_action(self, *, an: None | str = None) -> tuple[bool, bool]:
        return self._random_bool(), self._random_bool()

    def enum_action(self, *, an: None | str = None) -> MyEnum:
        return random.choice(list(MyEnum))

    bool_action.__action__ = ActionMetadata()  # type: ignore
    str_action.__action__ = ActionMetadata()  # type: ignore
    tuple_action.__action__ = ActionMetadata()  # type: ignore
    enum_action.__action__ = ActionMetadata()  # type: ignore
