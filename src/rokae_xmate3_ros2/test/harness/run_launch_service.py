#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-file", required=True)
    parser.add_argument("--launch-arg", action="append", default=[])
    args = parser.parse_args()

    launch_arguments = []
    for item in args.launch_arg:
        if ":=" not in item:
            parser.error(f"invalid --launch-arg value: {item}")
        key, value = item.split(":=", 1)
        launch_arguments.append((key, value))

    launch_service = launch.LaunchService(argv=[])
    launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(args.launch_file),
            launch_arguments=launch_arguments,
        )
    )
    return launch_service.run()


if __name__ == "__main__":
    sys.exit(main())
