#!/usr/bin/env python3

"""An example of the main script where its structure is generated by the Build
service for the specific project."""

from arcor2.data.common import uid
from arcor2.exceptions.runtime import print_exception
from arcor2.resources import Resources
from arcor2_kinali.object_types.abstract_simple import Settings as SimpleSettings
from arcor2_kinali.object_types.statistic import Statistic


def main(res: Resources) -> None:

    statistic = Statistic(uid(), "Whatever", SimpleSettings("http://127.0.0.1:16000"))
    statistic.add_value(res.project.id, "value_name", 1.0, an="action_name")


if __name__ == "__main__":
    try:
        with Resources("project_uuid") as res:
            main(res)
    except Exception as e:
        print_exception(e)
