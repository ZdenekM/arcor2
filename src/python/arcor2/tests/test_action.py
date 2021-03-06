import io

from arcor2.action import patch_object_actions
from arcor2.data.common import ActionMetadata
from arcor2.data.events import ActionState
from arcor2.object_types.abstract import Generic


def test_patch_object_actions(monkeypatch, capsys) -> None:
    class MyObject(Generic):
        def action(self) -> None:
            pass

        action.__action__ = ActionMetadata()  # type: ignore

    # @action tries to read from stdin
    sio = io.StringIO()
    sio.fileno = lambda: 0  # type: ignore  # fake whatever fileno
    monkeypatch.setattr("sys.stdin", sio)

    obj_id = "123"

    my_obj = MyObject(obj_id, "")

    my_obj.action()
    out_before, _ = capsys.readouterr()
    assert not out_before

    patch_object_actions(MyObject)

    my_obj.action()
    out_after, _ = capsys.readouterr()

    arr = out_after.strip().split("\n")
    assert len(arr) == 2

    before_evt = ActionState.from_json(arr[0])
    after_evt = ActionState.from_json(arr[1])

    assert before_evt.data.object_id == obj_id
    assert before_evt.data.method == MyObject.action.__name__
    assert before_evt.data.where == ActionState.Data.StateEnum.BEFORE

    assert after_evt.data.object_id == obj_id
    assert after_evt.data.method == MyObject.action.__name__
    assert after_evt.data.where == ActionState.Data.StateEnum.AFTER
