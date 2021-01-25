import select
import sys
from functools import wraps
from typing import Any, Callable, Dict, List, Optional, Type, TypeVar, Union, cast

from arcor2.cached import CachedProject, CachedScene
from arcor2.data.events import ActionStateAfter, ActionStateBefore, Event, PackageState
from arcor2.object_types.abstract import Generic
from arcor2.object_types.utils import iterate_over_actions
from arcor2.parameter_plugins.utils import plugin_from_instance

HANDLE_ACTIONS = True


def get_action_name_to_id(scene: CachedScene, project: CachedProject, object_type: str) -> Dict[str, str]:
    return {act.name: act.id for act in project.actions if scene.object(act.parse_type().obj_id).type == object_type}


def patch_object_actions(type_def: Type[Generic], action_name_to_id: Dict[str, str]) -> None:
    """Dynamically adds @action decorator to the methods with assigned
    ActionMetadata.

    :param type_def:
    :return:
    """

    # we somehow need to make action name->id accessible within the @action decorator
    type_def.action_name_to_id = action_name_to_id

    for method_name, method in iterate_over_actions(type_def):
        setattr(type_def, method_name, action(method))


try:
    # Windows solution
    import msvcrt
    import time

    def read_stdin(timeout: float = 0.0) -> Union[str, None]:

        time_to_end = time.monotonic() + timeout

        while True:

            x = msvcrt.kbhit()  # type: ignore
            if x:
                return msvcrt.getch().decode()  # type: ignore

            if not timeout or time.monotonic() > time_to_end:
                return None

            time.sleep(timeout / 100.0)


except ImportError:

    # Linux solution
    def read_stdin(timeout: float = 0.0) -> Union[str, None]:

        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.readline().strip()
        return None


def handle_action() -> None:

    ctrl_cmd = read_stdin()

    if ctrl_cmd == "p":
        print_event(PackageState(PackageState.Data(PackageState.Data.StateEnum.PAUSED)))
        while True:
            ctrl_cmd = read_stdin(0.1)
            if ctrl_cmd == "r":
                print_event(PackageState(PackageState.Data(PackageState.Data.StateEnum.RUNNING)))
                break


def print_event(event: Event) -> None:
    """Used from main script to print event as JSON."""

    print(event.to_json())
    sys.stdout.flush()


F = TypeVar("F", bound=Callable[..., Any])


def results_to_json(res: Any) -> Optional[List[str]]:
    """Prepares action results into list of JSONs. Return value could be tuple
    or single value.

    :param res:
    :return:
    """

    if res is None:
        return None

    if isinstance(res, tuple):
        return [plugin_from_instance(r).value_to_json(r) for r in res]
    else:
        return [plugin_from_instance(res).value_to_json(res)]


def action(f: F) -> F:
    @wraps(f)
    def wrapper(*args: Union[Generic, Any], **kwargs: Any) -> Any:

        # automagical overload for dictionary (allow to get rid of ** in script).
        if len(args) == 2 and isinstance(args[1], dict) and not kwargs:
            kwargs = args[1]
            args = (args[0],)

        action_name = args[1]
        action_args = args[2:]

        action_id = args[0].action_name_to_id[action_name]

        if not action.inside_composite and HANDLE_ACTIONS:  # type: ignore
            # TODO support also kwargs parameters
            print_event(
                ActionStateBefore(
                    ActionStateBefore.Data(
                        action_id, [plugin_from_instance(arg).value_to_json(arg) for arg in action_args]
                    )
                )
            )
            handle_action()

        if wrapper.__action__.composite:  # type: ignore # TODO and not step_into
            action.inside_composite = f  # type: ignore

        res = f(*args, **kwargs)

        if action.inside_composite == f:  # type: ignore
            action.inside_composite = None  # type: ignore

        if not action.inside_composite and HANDLE_ACTIONS:  # type: ignore
            print_event(ActionStateAfter(ActionStateAfter.Data(action_id, results_to_json(res))))
            handle_action()

        return res

    return cast(F, wrapper)


action.inside_composite = None  # type: ignore
