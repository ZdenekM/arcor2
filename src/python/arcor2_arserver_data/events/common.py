from dataclasses import dataclass
from typing import Optional

from dataclasses_jsonschema import JsonSchemaMixin

from arcor2.data import common
from arcor2.data.events import Event


@dataclass
class ShowMainScreen(Event):
    @dataclass
    class Data(JsonSchemaMixin):
        class WhatEnum(common.StrEnum):
            ScenesList = "ScenesList"
            ProjectsList = "ProjectsList"
            PackagesList = "PackagesList"

        what: WhatEnum
        highlight: Optional[str] = None

    data: Data


@dataclass
class ProcessState(Event):
    @dataclass
    class Data(JsonSchemaMixin):
        class StateEnum(common.StrEnum):
            Started = "started"
            Finished = "finished"
            Failed = "failed"

        id: str
        state: StateEnum
        message: Optional[str] = None

    data: Data
