#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys
import zipfile
from pathlib import Path


REQUIRED_WORKSPACE_RELATIVE_PATHS = (
    ".gitignore",
    "colcon_defaults.yaml",
)

REQUIRED_PACKAGE_RELATIVE_PATHS = (
    "src/runtime/owner_arbiter.hpp",
    "urdf/xMate3.xacro",
)

FORBIDDEN_DIR_PARTS = {
    "__pycache__",
    "build",
    "log",
    ".cache",
    "CMakeFiles",
    "Testing",
}

FORBIDDEN_FILE_SUFFIXES = (
    ".pyc",
    ".pdf",
)

FORBIDDEN_NAME_PREFIXES = (
    "tmp_",
)

XACRO_MARKERS = (
    'name="flange"',
    'name="tool0"',
    'name="tcp"',
    'name="payload"',
)


def _normalize_relative_path(path: str) -> str:
    return path.replace("\\", "/").strip("/")


def _load_tree_file(root: Path, relative_path: str) -> str:
    target = root / relative_path
    if not target.exists():
        raise FileNotFoundError(f"missing required source file: {target}")
    return target.read_text(encoding="utf-8")


def _load_archive_file(archive: zipfile.ZipFile, relative_path: str) -> str:
    normalized_target = _normalize_relative_path(relative_path)
    archive_entries = {_normalize_relative_path(name): name for name in archive.namelist()}
    matched = archive_entries.get(normalized_target)
    if matched is None:
        raise FileNotFoundError(f"archive is missing required file: {relative_path}")
    return archive.read(matched).decode("utf-8")


def _verify_markers(label: str, content: str) -> None:
    missing = [marker for marker in XACRO_MARKERS if marker not in content]
    if missing:
        raise RuntimeError(f"{label} is missing frame markers: {', '.join(missing)}")


def _verify_exact_match(label: str, expected: str, actual: str) -> None:
    # [OPTIMIZATION] Cross-platform immunity: Normalize CRLF (Windows) to LF (Linux)
    normalized_expected = expected.replace("\r\n", "\n")
    normalized_actual = actual.replace("\r\n", "\n")
    if normalized_expected != normalized_actual:
        raise RuntimeError(f"{label} does not match the working tree")


def _is_forbidden_archive_entry(path: str) -> bool:
    normalized = _normalize_relative_path(path)
    parts = [part for part in normalized.split("/") if part and part != "."]
    if any(part in FORBIDDEN_DIR_PARTS for part in parts):
        return True
    if any(Path(part).name.startswith(prefix) for part in parts for prefix in FORBIDDEN_NAME_PREFIXES):
        return True
    return any(normalized.endswith(suffix) for suffix in FORBIDDEN_FILE_SUFFIXES)


def _verify_archive_cleanliness(archive_namelist: list[str]) -> None:
    offenders = [name for name in archive_namelist if _is_forbidden_archive_entry(name)]
    if offenders:
        preview = ", ".join(sorted(offenders)[:8])
        raise RuntimeError(
            "archive appears to be a workspace snapshot, not a verified release archive: "
            f"{preview}"
        )


def _enforce_clean_env_if_requested() -> None:
    if os.environ.get("ROKAE_ENFORCE_CLEAN_ENV") != "1":
        return
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix and os.environ.get("ROKAE_CLEAN_ENV_ACTIVE") != "1":
        raise RuntimeError(
            "release archive verification must run from clean_build_env.sh; "
            f"active Conda environment detected at {conda_prefix}"
        )


def main() -> int:
    _enforce_clean_env_if_requested()
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-root", required=True)
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--archive", required=True)
    args = parser.parse_args()

    package_root = Path(args.package_root).resolve()
    workspace_root = Path(args.workspace_root).resolve()
    archive_path = Path(args.archive).resolve()

    if not package_root.is_dir():
        raise FileNotFoundError(f"package root does not exist: {package_root}")
    if not workspace_root.is_dir():
        raise FileNotFoundError(f"workspace root does not exist: {workspace_root}")
    if not archive_path.is_file():
        raise FileNotFoundError(f"archive does not exist: {archive_path}")

    with zipfile.ZipFile(archive_path) as archive:
        for relative_path in REQUIRED_WORKSPACE_RELATIVE_PATHS:
            tree_content = _load_tree_file(workspace_root, relative_path)
            archive_content = _load_archive_file(archive, relative_path)
            _verify_exact_match(f"workspace sidecar {relative_path}", tree_content, archive_content)

        for relative_path in REQUIRED_PACKAGE_RELATIVE_PATHS:
            tree_content = _load_tree_file(package_root, relative_path)
            archive_content = _load_archive_file(archive, relative_path)
            _verify_exact_match(f"package file {relative_path}", tree_content, archive_content)

        tree_xacro = _load_tree_file(package_root, "urdf/xMate3.xacro")
        archive_xacro = _load_archive_file(archive, "urdf/xMate3.xacro")
        
        _verify_markers("working tree xacro", tree_xacro)
        _verify_markers("archive xacro", archive_xacro)
        print("required content present")
        
        _verify_archive_cleanliness(archive.namelist())
    print("forbidden artifacts absent")

    print(f"archive consistency verified: {archive_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - command line failure path
        print(f"verify_source_archive.py: {exc}", file=sys.stderr)
        raise
