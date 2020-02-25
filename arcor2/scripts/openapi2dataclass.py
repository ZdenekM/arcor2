#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import horast
import typed_ast.ast3 as ast

import dataclasses
from dataclasses import dataclass, field
import dataclasses_jsonschema
from dataclasses_jsonschema import JsonSchemaMixin

from arcor2.source import utils as source_utils

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


def main():

    with open("project-swagger.json", "r") as api_file:
        api = json.loads(api_file.read())

    mm = ast.Module(body=[])
    source_utils.add_import(mm, dataclasses_jsonschema.__name__, JsonSchemaMixin.__name__)
    source_utils.add_import(mm, dataclasses.__name__, dataclass.__name__)
    source_utils.add_import(mm, dataclasses.__name__, field.__name__)

    # first, let's generate models
    for name, value in api["components"]["schemas"].items():

        if value["type"] != "object":
            # print(("not object", prop, prop_value))
            continue

        cls = dataclass_def(name)

        # TODO array/list, obj. reference
        for prop, prop_value in value["properties"].items():
            try:
                add_member(cls, prop, TYPE_MAPPING[prop_value["type"]].__name__, prop_value.get("description", ""))
            except KeyError as e:
                print((e, prop, prop_value))

        mm.body.append(cls)

    print()
    print(source_utils.tree_to_str(mm))


if __name__ == '__main__':
    main()
