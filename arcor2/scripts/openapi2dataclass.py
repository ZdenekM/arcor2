#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import typed_ast.ast3 as ast
from typing import List
import inspect
from enum import Enum

import dataclasses
from dataclasses import dataclass, field
import dataclasses_jsonschema
from dataclasses_jsonschema import JsonSchemaMixin

from arcor2.source import utils as source_utils
from arcor2 import helpers
from arcor2.data.common import StrEnum

import arcor2.data.common
import arcor2.data.object_type
import arcor2.data.robot
import arcor2.data.services

ARCOR2_DATA_CLASSES = {}


def dataclass_def(name: str, parent=None) -> ast.ClassDef:

    if parent is None:
        parent = JsonSchemaMixin.__name__

    return ast.ClassDef(
            name=name,
            bases=[ast.Name(
                id=parent,
                ctx=ast.Load())],
            keywords=[],
            body=[],
            decorator_list=[ast.Name(
                id=dataclass.__name__,
                ctx=ast.Load())])


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


def cls_def(schemas, done, mm, name: str) -> None:

    if name in done:
        return

    # add just import for known types (Pose, etc.)
    if name in ARCOR2_DATA_CLASSES:
        # TODO check if arcor2 version has all properties as the openapi version (arcor2 one may have more properties)
        done.add(name)
        print(name)
        module, cls_def = ARCOR2_DATA_CLASSES[name]
        source_utils.add_import(mm, module.__name__, cls_def.__name__)
        return

    value = schemas[name]

    if value["type"] == "string" and "enum" in value:
        source_utils.add_import(mm, arcor2.data.common.__name__, StrEnum.__name__)
        cls = enum_class_def(name, value["enum"])
        mm.body.append(cls)
        done.add(name)
        return

    if value["type"] != "object":
        raise SkipException

    cls = dataclass_def(name)

    # TODO array/list
    for prop, prop_value in value["properties"].items():

        if "type" in prop_value:

            # TODO handle array
            if prop_value["type"] in {"array", "object"}:
                continue

            # primitive types
            add_member(cls, helpers.camel_case_to_snake_case(prop),
                       TYPE_MAPPING[prop_value["type"]].__name__, prop_value.get("description", ""))

        elif "allOf" in prop_value:

            # handle complex stuff here
            # TODO support more items in allOf? at least print warning that something is ignored
            ref_name = prop_value["allOf"][0]["$ref"].split("/")[-1]

            if ref_name not in done:
                cls_def(schemas, done, mm, ref_name)

            assert ref_name in done

            add_member(cls, helpers.camel_case_to_snake_case(prop),
                       ref_name, prop_value.get("description", ""))

        else:
            print(f"special case: {prop_value}")

    mm.body.append(cls)
    done.add(name)


def main():

    with open("project-swagger.json", "r") as api_file:
        api = json.loads(api_file.read())

    for module in (arcor2.data.common, arcor2.data.object_type, arcor2.data.robot, arcor2.data.services):

        for name, obj in inspect.getmembers(module):
            if not inspect.isclass(obj) or not issubclass(obj, (JsonSchemaMixin, Enum)):
                continue

            ARCOR2_DATA_CLASSES[name] = module, obj

    mm = ast.Module(body=[])
    source_utils.add_import(mm, dataclasses_jsonschema.__name__, JsonSchemaMixin.__name__)
    source_utils.add_import(mm, dataclasses.__name__, dataclass.__name__)
    source_utils.add_import(mm, dataclasses.__name__, field.__name__)

    done = set()

    schemas = api["components"]["schemas"]

    for name in schemas.keys():

        try:
            cls_def(schemas, done, mm, name)
        except SkipException:
            print(f"Skipping {name}")
            continue

    print(source_utils.tree_to_str(mm))


if __name__ == '__main__':
    main()
