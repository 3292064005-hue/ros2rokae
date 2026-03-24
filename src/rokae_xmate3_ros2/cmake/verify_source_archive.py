#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path


REQUIRED_RELATIVE_PATHS = (
    ".gitignore",
    "src/rokae_xmate3_ros2/src/runtime/owner_arbiter.hpp",
    "src/rokae_xmate3_ros2/urdf/xMate3.xacro",
)

FORBIDDEN_DIR_PARTS = {
    "__pycache__",
    "build",
    "log",
    ".cache",
    "CMakeFiles",
}

FORBIDDEN_FILE_SUFFIXES = (
    ".pyc",
)

XACRO_MARKERS = (
    'name="flange"',
    'name="tool0"',
    'name="tcp"',
    'name="payload"',
)


def _normalize_relative_path(path: str) -> str:
    return path.replace("\\", "/").strip("/")


def _load_tree_file(source_root: Path, relative_path: str) -> str:
    target = source_root / relative_path
    if not target.exists():
        raise FileNotFoundError(f"missing required source file: {target}")
    return target.read_text(encoding="utf-8")


def _load_archive_file(archive_path: Path, relative_path: str) -> str:
    normalized_target = _normalize_relative_path(relative_path)
    with zipfile.ZipFile(archive_path) as archive:
        archive_entries = {_normalize_relative_path(name): name for name in archive.namelist()}
        matched = archive_entries.get(normalized_target)
        if matched is None:
            raise FileNotFoundError(f"archive is missing required file: {relative_path}")
        return archive.read(matched).decode("utf-8")


def _verify_markers(label: str, content: str) -> None:
    missing = [marker for marker in XACRO_MARKERS if marker not in content]
    if missing:
        raise RuntimeError(f"{label} is missing frame markers: {', '.join(missing)}")


def _is_forbidden_archive_entry(path: str) -> bool:
    normalized = _normalize_relative_path(path)
    parts = [part for part in normalized.split("/") if part and part != "."]
    if any(part in FORBIDDEN_DIR_PARTS for part in parts):
        return True
    return any(normalized.endswith(suffix) for suffix in FORBIDDEN_FILE_SUFFIXES)


def _verify_archive_cleanliness(archive_path: Path) -> None:
    with zipfile.ZipFile(archive_path) as archive:
        offenders = [name for name in archive.namelist() if _is_forbidden_archive_entry(name)]
    if offenders:
        preview = ", ".join(sorted(offenders)[:8])
        raise RuntimeError(f"archive contains forbidden artifacts: {preview}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--archive", required=True)
    args = parser.parse_args()

    source_root = Path(args.source_root).resolve()
    archive_path = Path(args.archive).resolve()

    if not source_root.is_dir():
        raise FileNotFoundError(f"source root does not exist: {source_root}")
    if not archive_path.is_file():
        raise FileNotFoundError(f"archive does not exist: {archive_path}")

    for relative_path in REQUIRED_RELATIVE_PATHS:
        _load_tree_file(source_root, relative_path)
        _load_archive_file(archive_path, relative_path)

    tree_xacro = _load_tree_file(source_root, "src/rokae_xmate3_ros2/urdf/xMate3.xacro")
    archive_xacro = _load_archive_file(archive_path, "src/rokae_xmate3_ros2/urdf/xMate3.xacro")
    _verify_markers("working tree xacro", tree_xacro)
    _verify_markers("archive xacro", archive_xacro)
    print("required content present")
    _verify_archive_cleanliness(archive_path)
    print("forbidden artifacts absent")

    print(f"archive consistency verified: {archive_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - command line failure path
        print(f"verify_source_archive.py: {exc}", file=sys.stderr)
        raise
