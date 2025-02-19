import copy
from ast import (
    AST,
    AnnAssign,
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
    NodeVisitor,
    Pass,
    Return,
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

import humps

import arcor2.data.common
from arcor2.cached import CachedProject
from arcor2.data.common import ActionPoint, Pose, Position, ProjectRobotJoints
from arcor2.exceptions import Arcor2Exception
from arcor2.source import SourceException
from arcor2.source.utils import add_import, find_function, tree_to_str


class SpecialValues:
    poses = "poses"
    joints = "joints"


RES_MODULE = "arcor2_runtime.resources"
RES_CLS = "Resources"


def main_loop(tree: Module) -> While:
    main = find_function("main", tree)

    for node in main.body:
        if isinstance(node, While):  # TODO more specific condition (test for True argument)
            return node

    raise SourceException("Main loop not found.")


def empty_script_tree(project_id: str, add_main_loop: bool = True) -> Module:
    """Creates barebones of the script (empty 'main' function).

    Returns
    -------
    """

    main_body: list[stmt] = [
        Assign(
            targets=[Name(id="aps", ctx=Store())],
            value=Call(func=Name(id="ActionPoints", ctx=Load()), args=[Name(id="res", ctx=Load())], keywords=[]),
            type_comment=None,
        )
    ]

    if add_main_loop:
        main_body.append(While(test=NameConstant(value=True, kind=None), body=[Pass()], orelse=[]))
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
                    posonlyargs=[],
                    args=[arg(arg="res", annotation=Name(id=RES_CLS, ctx=Load()), type_comment=None)],
                    vararg=None,
                    kwonlyargs=[],
                    kw_defaults=[],
                    kwarg=None,
                    defaults=[],
                ),
                body=main_body,
                decorator_list=[],
                returns=NameConstant(value=None, kind=None),
                type_comment=None,
                type_params=[],
            ),
            If(
                test=Compare(
                    left=Name(id="__name__", ctx=Load()), ops=[Eq()], comparators=[Str(value="__main__", kind="")]
                ),
                body=[
                    Try(
                        body=[
                            With(
                                items=[
                                    withitem(
                                        context_expr=Call(
                                            func=Name(id=RES_CLS, ctx=Load()),
                                            args=[],
                                            keywords=[],
                                        ),
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
                                type=Name(id=Exception.__name__, ctx=Load()),
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

    add_import(tree, "arcor2_runtime.exceptions", "print_exception", try_to_import=False)
    add_import(tree, RES_MODULE, RES_CLS, try_to_import=False)
    add_import(tree, "action_points", "ActionPoints", try_to_import=False)

    return tree


def global_action_points_class(project: CachedProject) -> str:
    tree = Module(body=[], type_ignores=[])

    tree.body.append(
        ImportFrom(
            module=arcor2.data.common.__name__,
            names=[
                alias(name=ActionPoint.__name__, asname=None),
                alias(name=Position.__name__, asname=None),
                alias(name=Pose.__name__, asname=None),
                alias(name=ProjectRobotJoints.__name__, asname=None),
            ],
            level=0,
        )
    )

    tree.body.append(
        ImportFrom(
            module=copy.__name__,
            names=[alias(name=copy.deepcopy.__name__, asname=None)],
            level=0,
        )
    )

    tree.body.append(
        ImportFrom(
            module=RES_MODULE,
            names=[alias(name=RES_CLS, asname=None)],
            level=0,
        )
    )

    aps_init_body: list[stmt] = []

    for ap in project.action_points:
        ap_cls_body: list[stmt] = [
            Assign(
                targets=[Attribute(value=Name(id="self", ctx=Load()), attr="_position", ctx=Store())],
                value=Attribute(
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr=CachedProject.bare_action_point.__name__,
                            ctx=Load(),
                        ),
                        args=[Str(value=ap.id, kind="")],
                        keywords=[],
                    ),
                    attr="position",
                    ctx=Load(),
                ),
                type_comment=None,
            )
        ]

        ap_type_name = humps.pascalize(ap.name)

        ap_joints_init_body: list[stmt] = []

        for joints in project.ap_joints(ap.id):
            ap_joints_init_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=f"_{joints.name}", ctx=Store())],
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr=SpecialValues.joints,
                            ctx=Load(),
                        ),
                        args=[Str(value=joints.id, kind="")],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        if ap_joints_init_body:
            ap_joints_cls_def = ClassDef(
                name=f"{ap_type_name}Joints",
                bases=[],
                keywords=[],
                body=[
                    FunctionDef(
                        name="__init__",
                        args=arguments(
                            posonlyargs=[],
                            args=[
                                arg(arg="self", annotation=None, type_comment=None),
                                arg(arg="res", annotation=Name(id=RES_CLS, ctx=Load()), type_comment=None),
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
                        type_params=[],
                    )
                ],
                decorator_list=[],
                type_params=[],
            )

            for joints in project.ap_joints(ap.id):
                ap_joints_cls_def.body.append(
                    FunctionDef(
                        name=joints.name,
                        args=arguments(
                            posonlyargs=[],
                            args=[arg(arg="self", annotation=None, type_comment=None)],
                            vararg=None,
                            kwonlyargs=[],
                            kw_defaults=[],
                            kwarg=None,
                            defaults=[],
                        ),
                        body=[
                            Return(
                                value=Call(
                                    func=Name(id=copy.deepcopy.__name__, ctx=Load()),
                                    args=[
                                        Attribute(value=Name(id="self", ctx=Load()), attr=f"_{joints.name}", ctx=Load())
                                    ],
                                    keywords=[],
                                )
                            )
                        ],
                        decorator_list=[Name(id="property", ctx=Load())],
                        returns=Name(id=ProjectRobotJoints.__name__, ctx=Load()),
                        type_comment=None,
                        type_params=[],
                    )
                )

            tree.body.append(ap_joints_cls_def)

            ap_cls_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=SpecialValues.joints, ctx=Store())],
                    value=Call(
                        func=Name(id=f"{ap_type_name}Joints", ctx=Load()),
                        args=[Name(id="res", ctx=Load())],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        ap_orientations_init_body: list[stmt] = []

        for ori in project.ap_orientations(ap.id):
            ap_orientations_init_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=f"_{ori.name}", ctx=Store())],
                    value=Call(
                        func=Attribute(
                            value=Attribute(value=Name(id="res", ctx=Load()), attr="project", ctx=Load()),
                            attr="pose",
                            ctx=Load(),
                        ),
                        args=[Str(value=ori.id, kind="")],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        if ap_orientations_init_body:
            ap_orientations_cls_def = ClassDef(
                name=f"{ap_type_name}Poses",
                bases=[],
                keywords=[],
                body=[
                    FunctionDef(
                        name="__init__",
                        args=arguments(
                            posonlyargs=[],
                            args=[
                                arg(arg="self", annotation=None, type_comment=None),
                                arg(arg="res", annotation=Name(id=RES_CLS, ctx=Load()), type_comment=None),
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
                        type_params=[],
                    )
                ],
                decorator_list=[],
                type_params=[],
            )

            for ori in project.ap_orientations(ap.id):
                ap_orientations_cls_def.body.append(
                    FunctionDef(
                        name=ori.name,
                        args=arguments(
                            posonlyargs=[],
                            args=[arg(arg="self", annotation=None, type_comment=None)],
                            vararg=None,
                            kwonlyargs=[],
                            kw_defaults=[],
                            kwarg=None,
                            defaults=[],
                        ),
                        body=[
                            Return(
                                value=Call(
                                    func=Name(id=copy.deepcopy.__name__, ctx=Load()),
                                    args=[
                                        Attribute(value=Name(id="self", ctx=Load()), attr=f"_{ori.name}", ctx=Load())
                                    ],
                                    keywords=[],
                                )
                            )
                        ],
                        decorator_list=[Name(id="property", ctx=Load())],
                        returns=Name(id=Pose.__name__, ctx=Load()),
                        type_comment=None,
                        type_params=[],
                    )
                )

            tree.body.append(ap_orientations_cls_def)

            ap_cls_body.append(
                Assign(
                    targets=[Attribute(value=Name(id="self", ctx=Load()), attr=SpecialValues.poses, ctx=Store())],
                    value=Call(
                        func=Name(id=f"{ap_type_name}Poses", ctx=Load()),
                        args=[Name(id="res", ctx=Load())],
                        keywords=[],
                    ),
                    type_comment=None,
                )
            )

        ap_cls_def = ClassDef(
            name=ap_type_name,
            bases=[],
            keywords=[],
            body=[
                FunctionDef(
                    name="__init__",
                    args=arguments(
                        posonlyargs=[],
                        args=[
                            arg(arg="self", annotation=None, type_comment=None),
                            arg(arg="res", annotation=Name(id=RES_CLS, ctx=Load()), type_comment=None),
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
                    type_params=[],
                )
            ],
            decorator_list=[],
            type_params=[],
        )

        # add copy property for position
        ap_cls_def.body.append(
            FunctionDef(
                name="position",
                args=arguments(
                    posonlyargs=[],
                    args=[arg(arg="self", annotation=None, type_comment=None)],
                    vararg=None,
                    kwonlyargs=[],
                    kw_defaults=[],
                    kwarg=None,
                    defaults=[],
                ),
                body=[
                    Return(
                        value=Call(
                            func=Name(id=copy.deepcopy.__name__, ctx=Load()),
                            args=[Attribute(value=Name(id="self", ctx=Load()), attr="_position", ctx=Load())],
                            keywords=[],
                        )
                    )
                ],
                decorator_list=[Name(id="property", ctx=Load())],
                returns=Name(id=Position.__name__, ctx=Load()),
                type_comment=None,
                type_params=[],
            )
        )

        tree.body.append(ap_cls_def)

        aps_init_body.append(
            Assign(
                targets=[Attribute(value=Name(id="self", ctx=Load()), attr=ap.name, ctx=Store())],
                value=Call(func=Name(id=ap_type_name, ctx=Load()), args=[Name(id="res", ctx=Load())], keywords=[]),
                type_comment=None,
            )
        )

    if not aps_init_body:  # there are no action points
        aps_init_body.append(Pass())

    aps_cls_def = ClassDef(
        name="ActionPoints",
        bases=[],
        keywords=[],
        body=[
            FunctionDef(
                name="__init__",
                args=arguments(
                    posonlyargs=[],
                    args=[
                        arg(arg="self", annotation=None, type_comment=None),
                        arg(arg="res", annotation=Name(id=RES_CLS, ctx=Load()), type_comment=None),
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
                type_params=[],
            )
        ],
        decorator_list=[],
        type_params=[],
    )

    tree.body.append(aps_cls_def)
    return tree_to_str(tree)


def find_last_assign(tree: FunctionDef) -> int:
    """This is intended to find last assign to a variable in the generated
    script (without logic).

    :param tree:
    :return:
    """

    for body_idx, body_item in reversed(list(enumerate(tree.body))):
        if isinstance(body_item, (Assign, AnnAssign)):
            return body_idx

    raise Arcor2Exception("Assign not found.")


def find_While(tree: Module):
    if tree.body == []:
        return tree

    class FindWhile(NodeVisitor):
        def __init__(self) -> None:
            self.While_node: While

        def visit_While(self, node: While) -> None:
            self.While_node = node
            self.generic_visit(node)
            return

    ff = FindWhile()
    ff.visit(tree)

    return ff.While_node


def find_Call(tree: Module | AST) -> Call:
    class FindCall(NodeVisitor):
        def __init__(self) -> None:
            self.Call_node: Call

        def visit_Call(self, node: Call) -> None:
            self.Call_node = node
            return

    ff = FindCall()
    ff.visit(tree)

    return ff.Call_node


def find_Compare(tree: Module | AST):
    class FindCompare(NodeVisitor):
        def __init__(self) -> None:
            self.Compare_node: list[Compare] = []

        def visit_Compare(self, node: Compare) -> None:
            self.Compare_node.append(node)
            return

    ff = FindCompare()
    ff.visit(tree)

    return ff.Compare_node[0]
