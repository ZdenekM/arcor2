#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import typed_ast.ast3 as ast
from typing import List, Optional, Dict, Set
import inspect
from enum import Enum
import typing
import tempfile
import os

import requests
import dataclasses
from dataclasses import dataclass, field
import dataclasses_jsonschema
from dataclasses_jsonschema import JsonSchemaMixin
from openapi_spec_validator import validate_spec  # type: ignore

from arcor2.source import utils as source_utils
from arcor2 import helpers
from arcor2.data.common import StrEnum

import arcor2.data.common
import arcor2.data.object_type
import arcor2.data.robot
import arcor2.data.services
import arcor2.services.service
import arcor2.services.robot_service
import arcor2.action

from arcor2.services.service import Service
from arcor2.services.robot_service import RobotService
from arcor2.data.common import ActionMetadata
from arcor2.action import action
from arcor2 import rest

"""
TODO
move functions to source/dataclasses

"""

ARCOR2_DATA_CLASSES = {}
LIST_STR = str(List).split(".")[-1]
COMMON = "common"


def srv_def(name: str, description: Optional[str] = None, parent: Optional[str] = None) -> ast.ClassDef:

    if parent is None:
        parent = Service.__name__

    srv_name = f"{name.title()}{Service.__name__}"

    return ast.ClassDef(
            name=srv_name,
            bases=[ast.Name(
                id=parent,
                ctx=ast.Load())],
            keywords=[],
            body=[
                ast.Expr(value=ast.Str(
                    s=description,
                    kind='')),
                ast.FunctionDef(
                    name='__init__',
                    args=ast.arguments(
                        args=[
                            ast.arg(
                                arg='self',
                                annotation=None,
                                type_comment=None),
                            ast.arg(
                                arg='configuration_id',
                                annotation=ast.Name(
                                    id='str',
                                    ctx=ast.Load()),
                                type_comment=None)],
                        vararg=None,
                        kwonlyargs=[],
                        kw_defaults=[],
                        kwarg=None,
                        defaults=[]),
                    body=[
                        ast.Expr(value=ast.Call(
                            func=ast.Attribute(
                                value=ast.Call(
                                    func=ast.Name(
                                        id='super',
                                        ctx=ast.Load()),
                                    args=[
                                        ast.Name(
                                            id=srv_name,
                                            ctx=ast.Load()),
                                        ast.Name(
                                            id='self',
                                            ctx=ast.Load())],
                                    keywords=[]),
                                attr='__init__',
                                ctx=ast.Load()),
                            args=[ast.Name(
                                id='configuration_id',
                                ctx=ast.Load())],
                            keywords=[])),
                        ast.Expr(value=ast.Call(
                            func=ast.Attribute(
                                value=ast.Name(
                                    id='systems',
                                    ctx=ast.Load()),
                                attr='create',
                                ctx=ast.Load()),
                            args=[
                                ast.Name(
                                    id='URL',
                                    ctx=ast.Load()),
                                ast.Name(
                                    id='self',
                                    ctx=ast.Load())],
                            keywords=[]))],
                    decorator_list=[],
                    returns=None,
                    type_comment=None),
                ast.FunctionDef(
                    name='get_configuration_ids',
                    args=ast.arguments(
                        args=[],
                        vararg=None,
                        kwonlyargs=[],
                        kw_defaults=[],
                        kwarg=None,
                        defaults=[]),
                    body=[ast.Return(value=ast.Call(
                        func=ast.Attribute(
                            value=ast.Name(
                                id='systems',
                                ctx=ast.Load()),
                            attr='systems',
                            ctx=ast.Load()),
                        args=[ast.Name(
                            id='URL',
                            ctx=ast.Load())],
                        keywords=[]))],
                    decorator_list=[ast.Name(
                        id='staticmethod',
                        ctx=ast.Load())],
                    returns=ast.Subscript(
                        value=ast.Name(
                            id='Set',
                            ctx=ast.Load()),
                        slice=ast.Index(value=ast.Name(
                            id='str',
                            ctx=ast.Load())),
                        ctx=ast.Load()),
                    type_comment=None)],
            decorator_list=[])


def dataclass_def(name: str, description: Optional[str] = None, parent=None) -> ast.ClassDef:

    if parent is None:
        parent = JsonSchemaMixin.__name__

    cls = ast.ClassDef(
            name=name,
            bases=[ast.Name(
                id=parent,
                ctx=ast.Load())],
            keywords=[],
            body=[],
            decorator_list=[ast.Name(
                id=dataclass.__name__,
                ctx=ast.Load())])

    if description:
        cls.body.append(ast.Expr(value=ast.Str(
            s=f'{description}',  # TODO how to make multiline docstring?
            kind='')))

    return cls


def enum_class_def(name: str, values: List[str]) -> ast.ClassDef:

    cls = ast.ClassDef(
        name=name,
        bases=[ast.Name(
            id=StrEnum.__name__,
            ctx=ast.Load())],
        keywords=[],
        body=[],
        decorator_list=[])

    for v in values:
        cls.body.append(
            ast.AnnAssign(
              target=ast.Name(
                id=helpers.camel_case_to_snake_case(v).upper(),
                ctx=ast.Store()),
              annotation=ast.Name(
                id='str',
                ctx=ast.Load()),
              value=ast.Str(
                s=v,
                kind=''),
              simple=1))

    return cls


def get_field(description: str) -> ast.Call:

    return ast.Call(
        func=ast.Name(
            id='field',
            ctx=ast.Load()),
        args=[],
        keywords=[ast.keyword(
            arg='metadata',
            value=ast.Call(
                func=ast.Name(
                    id='dict',
                    ctx=ast.Load()),
                args=[],
                keywords=[ast.keyword(
                    arg='description',
                    value=ast.Str(
                        s=description,
                        kind=''))]))])


def add_list_member(cls: ast.ClassDef, name: str, description: str, item_type: str) -> None:

    cls.body.append(
        ast.AnnAssign(
            target=ast.Name(
                id=name,
                ctx=ast.Store()),
            annotation=ast.Subscript(
                value=ast.Name(
                    id='List',
                    ctx=ast.Load()),
                slice=ast.Index(value=ast.Name(
                    id=item_type,
                    ctx=ast.Load())),
                ctx=ast.Load()),
            value=get_field(description),
            simple=1)
    )


def add_member(cls: ast.ClassDef, name: str, value_type: str, description: str) -> None:

    cls.body.append(ast.AnnAssign(
        target=ast.Name(
            id=name,
            ctx=ast.Store()),
        annotation=ast.Name(
            id=value_type,
            ctx=ast.Load()),
        value=ast.Call(
            func=ast.Name(
                id='field',
                ctx=ast.Load()),
            args=[],
            keywords=[ast.keyword(
                arg='metadata',
                value=ast.Call(
                    func=ast.Name(
                        id='dict',
                        ctx=ast.Load()),
                    args=[],
                    keywords=[ast.keyword(
                        arg='description',
                        value=ast.Str(
                            s=description,
                            kind=''))]))]),
        simple=1))


TYPE_MAPPING = {"number": float,
               "string": str,
               "boolean": bool,
                "integer": int}


class SkipException(Exception):
    pass


def cls_def(api_name: str, modules, api_spec, done: Set[str], common_models: Set[str], model_name: str) -> None:

    if model_name in done:
        return

    # add just import for known types (Pose, etc.)
    if model_name in ARCOR2_DATA_CLASSES:
        # TODO check if arcor2 version has all properties as the openapi version (arcor2 one may have more properties)
        done.add(model_name)
        module, cls_def = ARCOR2_DATA_CLASSES[model_name]
        source_utils.add_import(modules[api_name], module.__name__, cls_def.__name__)
        return

    if model_name in common_models:
        mod = modules[COMMON]
        source_utils.add_import(modules[api_name], COMMON, model_name, try_to_import=False)
    else:
        mod = modules[api_name]

    value = api_spec[api_name]["components"]["schemas"][model_name]

    if "$ref" in value:

        # TODO cache/save it to dict
        ref_api = requests.get(value["$ref"]).json()
        # TODO process definition
        return

    if value["type"] == "string" and "enum" in value:
        source_utils.add_import(mod, arcor2.data.common.__name__, StrEnum.__name__)
        cls = enum_class_def(model_name, value["enum"])
        mod.body.append(cls)
        done.add(model_name)
        return

    if value["type"] != "object":
        raise SkipException

    cls = dataclass_def(model_name, value.get("description", None))

    for prop, prop_value in value["properties"].items():

        if "type" in prop_value:

            if prop_value["type"] == "object":
                # TODO print warning
                continue

            if prop_value["type"] == "array":

                source_utils.add_import(mod, typing.__name__, LIST_STR)

                add_list_member(cls, helpers.camel_case_to_snake_case(prop), prop_value.get("description", ""),
                                prop_value["items"]["$ref"].split("/")[-1])
                continue

            # primitive types
            add_member(cls, helpers.camel_case_to_snake_case(prop),
                       TYPE_MAPPING[prop_value["type"]].__name__, prop_value.get("description", ""))

        elif "allOf" in prop_value:

            # handle complex stuff here
            # TODO support more items in allOf? at least print warning that something is ignored
            ref_name = prop_value["allOf"][0]["$ref"].split("/")[-1]

            if ref_name not in done:
                cls_def(api_name, modules, api_spec, done, common_models, model_name)

            assert ref_name in done

            add_member(cls, helpers.camel_case_to_snake_case(prop),
                       ref_name, prop_value.get("description", ""))

        else:
            print(f"special case: {prop_value}")

    mod.body.append(cls)
    done.add(model_name)


def empty_data_module() -> ast.Module:

    mm = ast.Module(body=[])
    source_utils.add_import(mm, dataclasses_jsonschema.__name__, JsonSchemaMixin.__name__)
    source_utils.add_import(mm, dataclasses.__name__, dataclass.__name__)
    source_utils.add_import(mm, dataclasses.__name__, field.__name__)
    return mm


def name_to_srv_url(api_name: str) -> str:

    # TODO api_name to camel case
    return f"{api_name.upper()}_SERVICE_URL"


def empty_srv_module(api_name: str, robot: bool = False) -> ast.Module:

    mm = ast.Module(body=[])
    if robot:
        source_utils.add_import(mm, arcor2.services.robot_service.__name__, RobotService.__name__)
    else:
        source_utils.add_import(mm, arcor2.services.service.__name__, Service.__name__)

    source_utils.add_import(mm, arcor2.data.common.__name__, ActionMetadata.__name__)
    source_utils.add_import(mm, arcor2.action.__name__, action.__name__)
    source_utils.add_import(mm, arcor2.__name__, rest.__name__, try_to_import=False)  # TODO fix it!

    # TODO import os

    mm.body.append(ast.Assign(
            targets=[ast.Name(
                id='URL',
                ctx=ast.Store())],
            value=ast.Call(
                func=ast.Attribute(
                    value=ast.Name(
                        id=os.__name__,
                        ctx=ast.Load()),
                    attr=os.getenv.__name__,
                    ctx=ast.Load()),
                args=[ast.Str(
                    s=name_to_srv_url(api_name),
                    kind='')],
                keywords=[]),
            type_comment=None))

    return mm


def main():

    data_modules: Dict[str, ast.Module] = {}
    data_modules_names: Dict[str, Set[str]] = {}

    api_spec: Dict[str, Dict] = {}

    for module in (arcor2.data.common, arcor2.data.object_type, arcor2.data.robot, arcor2.data.services):

        for name, obj in inspect.getmembers(module):
            if not inspect.isclass(obj) or not issubclass(obj, (JsonSchemaMixin, Enum)):
                continue

            ARCOR2_DATA_CLASSES[name] = module, obj

    done = set()

    for api_file_name in ("robot-swagger.json", "search-swagger.json"):

        with open(api_file_name, "r") as api_file:
            api = json.loads(api_file.read())

        validate_spec(api)
        name = api_file_name.split("-")[0]
        data_modules[name] = empty_data_module()
        api_spec[name] = api

    model_name_cnt: Dict[str, int] = {}
    for api_name, module in data_modules.items():
        schemas = api_spec[api_name]["components"]["schemas"]

        for model_name in schemas.keys():

            if model_name in ARCOR2_DATA_CLASSES:
                continue

            if model_name not in model_name_cnt:
                model_name_cnt[model_name] = 1
            else:
                model_name_cnt[model_name] += 1

    common_models: Set[str] = {k for k, v in model_name_cnt.items() if v > 1}
    print(common_models)

    data_modules[COMMON] = empty_data_module()  # default module for shared dataclasses

    for api_name, api in api_spec.items():

        schemas = api["components"]["schemas"]
        for model_name in schemas.keys():

            try:
                cls_def(api_name, data_modules, api_spec, done, common_models, model_name)
                if api_name not in data_modules_names:  # TODO default dict
                    data_modules_names[api_name] = set()

                data_modules_names[api_name].add(model_name)

            except SkipException:
                print(f"Skipping {name}")
                continue

    srv_modules: Dict[str, ast.Module] = {}

    for api_name, api in api_spec.items():

        robot = api_name == "robot"
        srv_modules[api_name] = empty_srv_module(api_name, robot)
        srv_modules[api_name].body.append(srv_def(api_name, api["info"]["description"], RobotService.__name__ if robot else None))

        # add actions
        for path, path_value in api["paths"].items():

            for op, op_value in path_value.items():

                if "Server" in op_value["tags"]:  # just ignore
                    continue

                if "Systems" in op_value["tags"]:  # TODO generate systems
                    continue

                if "Utils" in op_value["tags"]:  # TODO focus
                    continue

                if "Collisions" in op_value["tags"]:  # TODO generate ordinary methods
                    continue

                if op not in {"put", "get"}:
                    # raise Exception(f"Unsupported type of http method: {op}")
                    continue

                method_name = helpers.camel_case_to_snake_case(op_value['operationId'])
                desc = op_value["summary"]

                print(f"path: {path}, op_id: {op_value['operationId']}, tags: {op_value['tags']}")

                if "parameters" in op_value:
                    for param in op_value["parameters"]:

                        name = param["name"]
                        desc = param.get("description", None)

                        if "type" in param["schema"]:
                            ptype = TYPE_MAPPING[param["schema"]["type"]].__name__
                        elif "$ref" in param["schema"]:

                            type_name = param["schema"]["$ref"].split("/")[-1]

                            if type_name in ARCOR2_DATA_CLASSES:
                                ptype = ARCOR2_DATA_CLASSES[type_name].__name__
                            elif type_name in common_models:
                                # TODO add import
                                ptype = type_name
                            elif type_name in data_modules_names[api_name]:
                                # TODO add import
                                ptype = type_name
                            else:
                                raise Exception(f"Unknown parameter type: {type_name}")

    # with tempfile.TemporaryDirectory() as tmp_dir:
    os.mkdir("generated_package")  # TODO get package name as argument?
    os.chdir("generated_package")
    os.mkdir("data")
    for module_name, module in data_modules.items():
        with open(os.path.join("data", module_name + ".py"), "w") as mod_file:
            mod_file.write(source_utils.tree_to_str(module))

    for api_name, api in api_spec.items():
        with open(api_name + ".py", "w") as api_file:
            api_file.write(source_utils.tree_to_str(srv_modules[api_name]))


if __name__ == '__main__':
    main()
