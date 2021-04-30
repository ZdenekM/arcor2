from dataclasses import dataclass
from arcor2 import rest
from arcor2.object_types.abstract import Settings


@dataclass
class UrlSettings(Settings):

    url: str


class FitCommonMixin:

    def _started(self) -> bool:
        return rest.call(rest.Method.GET, f"{self.settings.url}/started", return_type=bool)  # type: ignore

    def _stop(self) -> None:
        rest.call(rest.Method.PUT, f"{self.settings.url}/stop")  # type: ignore
