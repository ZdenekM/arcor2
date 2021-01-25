from typing import List

import humps
from typed_ast.ast3 import (
    Assign,
    Attribute,
    Call,
    ClassDef,
    Compare,
    Eq,
    ExceptHandler,
    Expr,
    FunctionDef,
    If,
    ImportFrom,
    Load,
    Module,
    Name,
    NameConstant,
    Pass,
    Store,
    Str,
    Try,
    While,
    With,
    alias,
    arg,
    arguments,
    stmt,
    withitem,
)

import arcor2.data.common
import arcor2.exceptions.runtime
import arcor2.resources
from arcor2.cached import CachedProject
from arcor2.data.common import ActionPoint
from arcor2.source import SourceException
from arcor2.source.utils import add_import, find_function, tree_to_str


def main_loop(tree: Module) -> While:
    main = find_function("main", tree)

    for node in main.body:
        if isinstance(node, While):  # TODO more specific condition (test for True argument)
            return node

    raise SourceException("Main loop not found.")


def empty_script_tree(add_main_loop: bool = True) -> Module:
    """Creates barebones of the script (empty 'main' function).

    Returns
    -------
    """

    main_body: List[stmt] = [
        Assign(
            targets=[Name(id="aps", ctx=Store())],
            value=Call(func=Name(id="ActionPoints", ctx=Load()), args=[Name(id="res", ctx=Load())], keywords=[]),
            type_comment=None,
        )
    ]

    if add_main_loop:
        main_body.append(While(test=NameConstant(value=True), body=[Pass()], orelse=[]))
    else:
        """put there "pass" in order to make code valid even if there is no
        other statement (e.g. no object from resources)"""
        main_body.append(Pass())

    # TODO helper function for try ... except

    tree = Module(
        body=[
            FunctionDef(
                name="main",
                args=arguments(
                    args=[arg(arg="res", annotation=Name(id="Resources", ctx=Load()), type_comment=None)],
                    vararg=None,
                    kwonlyargs=[],
                    kw_defaults=[],
                    kwarg=None,
                    defaults=[],
                ),
                body=main_body,
                decorator_list=[],
                returns=NameConstant(value=None),
                type_comment=None,
            ),
            If(
                test=Compare(
                    left=Name(id="__name__", ctx=Load()), ops=[Eq()], comparators=[Str(s="__main__", kind="")]
                ),
                body=[
                    Try(
                        body=[
                            With(
                                items=[
                                    withitem(
                                        context_expr=Call(func=Name(id="Resources", ctx=Load()), args=[], keywords=[]),
                                        optional_vars=Name(id="res", ctx=Store()),
                                    )
                                ],
                                body=[
                                    Expr(
                                        value=Call(
                                            func=Name(id="main", ctx=Load()),
                                            args=[Name(id="res", ctx=Load())],
                                            keywords=[],
                                        )
                                    )
                                ],
                                type_comment=None,
                            )
                        ],
                        handlers=[
                            ExceptHandler(
                                type=Name(id="Exception", ctx=Load()),
                                name="e",
                                body=[
                                    Expr(
                                        value=Call(
                                            func=Name(id="print_exception", ctx=Load()),
                                            args=[Name(id="e", ctx=Load())],
                                            keywords=[],
                                        )
                                    )
                                ],
                            )
                        ],
                        orelse=[],
                        finalbody=[],
                    )
                ],
                orelse=[],
            ),
        ],
        type_ignores=[],
    )

    add_import(tree, arcor2.exceptions.runtime.__name__, arcor2.exceptions.runtime.print_exception.__name__)
    add_import(tree, arcor2.resources.__name__, arcor2.resources.Resources.__name__, try_to_import=False)
    add_import(tree, "action_points", "ActionPoints", try_to_import=False)

    return tree


def global_action_points_class(project: CachedProject) -> str:
    tree = Module(body=[])
    tree.body.append(
        ImportFrom(module=arcor2.data.common.__name__, names=[alias(name=ActionPoint.__name__, asname=None)], level=0)
    )
    tree.body.append(ImportFrom(module="resources", names=[alias(name="Resources", asname=None)], level=0))

    aps_init_body = []

    for ap in project.action_points:

        ap_cls_body: List[Assign] = [
            Assign(
                targets=[Attribute(value=Name(id="self", ctx=Load()), attr="position", ctx=Store())],
                value=Attribute(
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr="bare_action_point",
                            ctx=Load(),
                        ),
                        args=[Str(s=ap.id, kind="")],
                        keywords=[],
                    ),
                    attr="position",
                    ctx=Load(),
                ),
                type_comment=None,
            )
        ]

        ap_type_name = humps.pascalize(ap.name)

        ap_joints_init_body: List[Assign] = []

        for joints in project.ap_joints(ap.id):
            ap_joints_init_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=joints.name, ctx=Store())],
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr="joints",
                            ctx=Load(),
                        ),
                        args=[Str(s=joints.id, kind="")],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        if ap_joints_init_body:

            tree.body.append(
                ClassDef(
                    name=f"{ap_type_name}Joints",
                    bases=[],
                    keywords=[],
                    body=[
                        FunctionDef(
                            name="__init__",
                            args=arguments(
                                args=[
                                    arg(arg="self", annotation=None, type_comment=None),
                                    arg(arg="res", annotation=Name(id="Resources", ctx=Load()), type_comment=None),
                                ],
                                vararg=None,
                                kwonlyargs=[],
                                kw_defaults=[],
                                kwarg=None,
                                defaults=[],
                            ),
                            body=ap_joints_init_body,
                            decorator_list=[],
                            returns=None,
                            type_comment=None,
                        )
                    ],
                    decorator_list=[],
                )
            )

            ap_cls_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr="joints", ctx=Store())],
                    value=Call(
                        func=Name(id=f"{ap_type_name}Joints", ctx=Load()),
                        args=[Name(id="res", ctx=Load())],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        ap_orientations_init_body: List[Assign] = []

        for ori in project.ap_orientations(ap.id):
            ap_orientations_init_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=ori.name, ctx=Store())],
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr="pose",
                            ctx=Load(),
                        ),
                        args=[Str(s=ori.id, kind="")],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        if ap_orientations_init_body:
            tree.body.append(
                ClassDef(
                    name=f"{ap_type_name}Poses",
                    bases=[],
                    keywords=[],
                    body=[
                        FunctionDef(
                            name="__init__",
                            args=arguments(
                                args=[
                                    arg(arg="self", annotation=None, type_comment=None),
                                    arg(arg="res", annotation=Name(id="Resources", ctx=Load()), type_comment=None),
                                ],
                                vararg=None,
                                kwonlyargs=[],
                                kw_defaults=[],
                                kwarg=None,
                                defaults=[],
                            ),
                            body=ap_orientations_init_body,
                            decorator_list=[],
                            returns=None,
                            type_comment=None,
                        )
                    ],
                    decorator_list=[],
                )
            )

            ap_cls_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr="poses", ctx=Store())],
                    value=Call(
                        func=Name(id=f"{ap_type_name}Poses", ctx=Load()),
                        args=[Name(id="res", ctx=Load())],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        tree.body.append(
            ClassDef(
                name=ap_type_name,
                bases=[],
                keywords=[],
                body=[
                    FunctionDef(
                        name="__init__",
                        args=arguments(
                            args=[
                                arg(arg="self", annotation=None, type_comment=None),
                                arg(arg="res", annotation=Name(id="Resources", ctx=Load()), type_comment=None),
                            ],
                            vararg=None,
                            kwonlyargs=[],
                            kw_defaults=[],
                            kwarg=None,
                            defaults=[],
                        ),
                        body=ap_cls_body,
                        decorator_list=[],
                        returns=None,
                        type_comment=None,
                    )
                ],
                decorator_list=[],
            )
        )

        aps_init_body.append(
            Assign(
                targets=[Attribute(value=Name(id="self", ctx=Load()), attr=ap.name, ctx=Store())],
                value=Call(func=Name(id=ap_type_name, ctx=Load()), args=[Name(id="res", ctx=Load())], keywords=[]),
                type_comment=None,
            )
        )

    if not aps_init_body:  # there are no action points
        aps_init_body.append(Pass)

    aps_cls_def = ClassDef(
        name="ActionPoints",
        bases=[],
        keywords=[],
        body=[
            FunctionDef(
                name="__init__",
                args=arguments(
                    args=[
                        arg(arg="self", annotation=None, type_comment=None),
                        arg(arg="res", annotation=Name(id="Resources", ctx=Load()), type_comment=None),
                    ],
                    vararg=None,
                    kwonlyargs=[],
                    kw_defaults=[],
                    kwarg=None,
                    defaults=[],
                ),
                body=aps_init_body,
                decorator_list=[],
                returns=None,
                type_comment=None,
            )
        ],
        decorator_list=[],
    )

    tree.body.append(aps_cls_def)
    return tree_to_str(tree)
