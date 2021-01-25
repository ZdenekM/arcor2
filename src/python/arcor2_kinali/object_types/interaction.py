from dataclasses import dataclass
from typing import List

from dataclasses_jsonschema import JsonSchemaMixin

from arcor2 import rest
from arcor2.data.common import ActionMetadata, StrEnum

from .abstract_simple import AbstractSimple


class NotificationLevelEnum(StrEnum):

    INFO: str = "Info"
    WARN: str = "Warn"
    ERROR: str = "Error"


@dataclass
class DialogValue(JsonSchemaMixin):

    title: str
    content: str
    options: List[str]


@dataclass
class NotificationValue(JsonSchemaMixin):

    message: str
    level: NotificationLevelEnum
    created: int


class Interaction(AbstractSimple):
    """REST interface to the Interaction service (0.1.1)."""

    _ABSTRACT = False

    # --- Dialog Controller --------------------------------------------------------------------------------------------

    def add_dialog(self, action_name: str, title: str, content: str, options: List[str]) -> str:
        """Creates dialog, block until operator selects one of the options.

        :param title: Dialog title
        :param content: Dialog content
        :param options: Dialog options.
        :return:
        """

        return rest.call(
            rest.Method.PUT,
            f"{self.settings.url}/dialog",
            return_type=str,
            params={"title": title, "content": content},
            body=options,
            timeout=rest.Timeout(read=8 * 60 * 60),
        )

    def get_dialog(self, action_name: str) -> DialogValue:
        """Returns active dialog (title and options) or error if no dialog is
        active.

        :return:
        """

        return rest.call(rest.Method.GET, f"{self.settings.url}/dialog", return_type=DialogValue)

    def resolve_dialog(self, action_name: str, option: str) -> None:
        """Resolves current dialog using one of the options.

        :param option:
        :return:
        """

        rest.call(rest.Method.PUT, f"{self.settings.url}/dialog/resolve", params={"option": option})

    # --- Notification Controller --------------------------------------------------------------------------------------

    def add_notification(self, action_name: str, message: str, level: NotificationLevelEnum) -> None:
        """Creates notification.

        :param message:
        :param level:
        :return:
        """

        rest.call(rest.Method.PUT, f"{self.settings.url}/notification", params={"message": message, "level": level})

    def delete_notifications(self, action_name: str) -> None:
        """Clears all notifications.

        :return:
        """

        rest.call(rest.Method.DELETE, f"{self.settings.url}/notifications")

    def get_notifications(self, action_name: str, since_timestamp: int) -> List[NotificationValue]:
        """Gets all notifications stored after given timestamp.

        :param since_timestamp: UNIX timestamp in nanoseconds, after which created notifications are returned.
        :return:
        """
        return rest.call(
            rest.Method.GET,
            f"{self.settings.url}/notifications",
            list_return_type=NotificationValue,
            params={"sinceTimestamp": since_timestamp},
        )

    add_dialog.__action__ = ActionMetadata(blocking=True)  # type: ignore
    get_dialog.__action__ = ActionMetadata(blocking=True)  # type: ignore
    resolve_dialog.__action__ = ActionMetadata(blocking=True)  # type: ignore
    add_notification.__action__ = ActionMetadata(blocking=True)  # type: ignore
    delete_notifications.__action__ = ActionMetadata(blocking=True)  # type: ignore
    get_notifications.__action__ = ActionMetadata(blocking=True)  # type: ignore
