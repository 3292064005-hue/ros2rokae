#!/usr/bin/env python3

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-file", required=True)
    parser.add_argument("--launch-arg", action="append", default=[])
    parser.add_argument("--shutdown-sentinel")
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

    stop_event = threading.Event()
    sentinel_path = Path(args.shutdown_sentinel) if args.shutdown_sentinel else None

    def _watch_shutdown_sentinel():
        if sentinel_path is None:
            return
        while not stop_event.is_set():
            if sentinel_path.exists():
                launch_service.shutdown(force_sync=True)
                return
            time.sleep(0.1)

    sentinel_thread = threading.Thread(target=_watch_shutdown_sentinel, daemon=True)
    sentinel_thread.start()

    def _request_shutdown(_signum, _frame):
        launch_service.shutdown(force_sync=True)

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    try:
        return launch_service.run()
    finally:
        stop_event.set()
        sentinel_thread.join(timeout=1.0)


if __name__ == "__main__":
    sys.exit(main())
