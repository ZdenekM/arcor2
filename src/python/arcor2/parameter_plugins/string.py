from typing import Any, List

from typed_ast.ast3 import Str

from arcor2.cached import CachedProject as CProject
from arcor2.cached import CachedScene as CScene
from arcor2.parameter_plugins.base import ParameterPlugin, TypesDict
from arcor2.parameter_plugins.list import ListParameterPlugin, get_type_name


class StringPlugin(ParameterPlugin):
    @classmethod
    def type(cls) -> Any:
        return str

    @classmethod
    def type_name(cls) -> str:
        return "string"

    @classmethod
    def parameter_value(
        cls, type_defs: TypesDict, scene: CScene, project: CProject, action_id: str, parameter_id: str
    ) -> str:
        return super(StringPlugin, cls).parameter_value(type_defs, scene, project, action_id, parameter_id)

    @classmethod
    def parameter_stmt(
        cls, type_defs: TypesDict, scene: CScene, project: CProject, action_id: str, parameter_id: str
    ) -> Str:
        return Str(s=cls.parameter_execution_value(type_defs, scene, project, action_id, parameter_id), kind="")


class StringListPlugin(ListParameterPlugin):
    @classmethod
    def type(cls):
        return List[str]

    @classmethod
    def type_name(cls) -> str:
        return get_type_name(StringPlugin)  # type: ignore

    @classmethod
    def parameter_value(
        cls, type_defs: TypesDict, scene: CScene, project: CProject, action_id: str, parameter_id: str
    ) -> List[str]:
        return super(StringListPlugin, cls).parameter_value(type_defs, scene, project, action_id, parameter_id)
