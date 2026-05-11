#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
WORKFLOW = ROOT / ".github" / "workflows" / "acceptance-humble-gazebo11.yml"


def require(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise SystemExit(message)


def main() -> int:
    if not WORKFLOW.is_file():
        raise SystemExit(f"missing workflow: {WORKFLOW}")
    text = WORKFLOW.read_text(encoding="utf-8")
    require(text, "pull_request:", "acceptance workflow must run on pull_request")
    require(text, "push:", "acceptance workflow must run on push")
    require(text, "--release-gate", "acceptance workflow must execute the locked release gate")
    require(text, "--launch-smoke", "acceptance workflow must execute launch smoke in the locked environment")
    require(text, "acceptance-humble-gazebo11-report", "acceptance workflow must upload the locked-env acceptance report")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
