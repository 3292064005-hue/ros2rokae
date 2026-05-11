#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

INTERNAL_SERVICES = {
    "GetAI": "get_ai",
    "GetAvoidSingularity": "get_avoid_singularity",
    "GetDI": "get_di",
    "GetDO": "get_do",
    "GetRlProjectInfo": "get_rl_project_info",
    "GetToolCatalog": "get_tool_catalog",
    "GetWobjCatalog": "get_wobj_catalog",
    "LoadRLProject": "load_rl_project",
    "PauseRLProject": "pause_rl_project",
    "PlannerPreflightReport": "planner_preflight_report",
    "ReadRegister": "read_register",
    "ReadRegisterEx": "read_register_ex",
    "RegisterDataCallback": "register_data_callback",
    "SendCustomData": "send_custom_data",
    "SetAO": "set_ao",
    "SetAvoidSingularity": "set_avoid_singularity",
    "SetDI": "set_di",
    "SetDO": "set_do",
    "SetProjectRunningOpt": "set_project_running_opt",
    "SetSimulationMode": "set_simulation_mode",
    "SetXPanelVout": "set_x_panel_vout",
    "StartRLProject": "start_rl_project",
    "StopRLProject": "stop_rl_project",
    "ValidateMotion": "validate_motion",
    "WriteRegister": "write_register",
    "WriteRegisterEx": "write_register_ex",
}

INTERNAL_ONLY_FILES = {
    Path("src/runtime/io_program_facade.cpp"),
    Path("src/runtime/query_catalog_service.cpp"),
}

SCAN_ROOTS = [
    ROOT / "include",
    ROOT / "src",
    ROOT / "test" / "unit",
]
SCAN_SUFFIXES = {".cpp", ".hpp", ".h"}

TYPE_RE = re.compile(r"rokae_xmate3_ros2::srv::(" + "|".join(sorted(INTERNAL_SERVICES)) + r")\b")
INCLUDE_RE = re.compile(
    r'#\s*include\s+[<"]rokae_xmate3_ros2/srv/('
    + "|".join(sorted(re.escape(name) for name in INTERNAL_SERVICES.values()))
    + r')\.hpp[>"]'
)
FACADE_RE = re.compile(r"\b(rt::)?IoProgramFacade\b")


def rel(path: Path) -> Path:
    return path.relative_to(ROOT)


def internal_guard_state(stack: list[dict[str, bool]]) -> bool:
    return any(frame["inside_internal"] for frame in stack if frame["is_internal_guard"])


def push_guard(line: str, stack: list[dict[str, bool]]) -> bool:
    stripped = line.strip()
    if stripped.startswith("#ifdef"):
        expr = stripped[len("#ifdef"):].strip()
        stack.append({
            "is_internal_guard": expr == "ROKAE_ENABLE_INTERNAL_SURFACE",
            "inside_internal": expr == "ROKAE_ENABLE_INTERNAL_SURFACE",
        })
        return True
    if stripped.startswith("#ifndef"):
        expr = stripped[len("#ifndef"):].strip()
        stack.append({
            "is_internal_guard": expr == "ROKAE_ENABLE_INTERNAL_SURFACE",
            "inside_internal": False,
        })
        return True
    if stripped.startswith("#if"):
        expr = stripped[len("#if"):].strip()
        is_internal = "ROKAE_ENABLE_INTERNAL_SURFACE" in expr
        inside_internal = is_internal and "!" not in expr and "== 0" not in expr
        stack.append({
            "is_internal_guard": is_internal,
            "inside_internal": inside_internal,
        })
        return True
    return False


def update_guard(line: str, stack: list[dict[str, bool]]) -> None:
    stripped = line.strip()
    if push_guard(stripped, stack):
        return
    if stripped.startswith("#elif") and stack:
        frame = stack[-1]
        if frame["is_internal_guard"]:
            expr = stripped[len("#elif"):].strip()
            frame["inside_internal"] = "ROKAE_ENABLE_INTERNAL_SURFACE" in expr and "!" not in expr and "== 0" not in expr
        return
    if stripped.startswith("#else") and stack:
        frame = stack[-1]
        if frame["is_internal_guard"]:
            frame["inside_internal"] = not frame["inside_internal"]
        return
    if stripped.startswith("#endif") and stack:
        stack.pop()


def iter_paths() -> list[Path]:
    paths: list[Path] = []
    for root in SCAN_ROOTS:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in SCAN_SUFFIXES:
                continue
            if rel(path) in INTERNAL_ONLY_FILES:
                continue
            paths.append(path)
    return sorted(paths)


def check_file(path: Path) -> list[str]:
    failures: list[str] = []
    stack: list[dict[str, bool]] = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        update_guard(line, stack)
        if internal_guard_state(stack):
            continue
        if TYPE_RE.search(line) or INCLUDE_RE.search(line):
            failures.append(f"{rel(path)}:{line_no}: internal service type/include is outside ROKAE_ENABLE_INTERNAL_SURFACE")
            continue
        if FACADE_RE.search(line) and "class IoProgramFacade;" not in line:
            failures.append(f"{rel(path)}:{line_no}: IoProgramFacade reference is outside ROKAE_ENABLE_INTERNAL_SURFACE")
    return failures


def main() -> int:
    failures: list[str] = []
    for path in iter_paths():
        failures.extend(check_file(path))
    if failures:
        print("public/internal boundary check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("public/internal boundary check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
