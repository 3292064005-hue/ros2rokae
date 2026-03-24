#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
import zipfile
from pathlib import Path


WORKSPACE_SIDECARS = (
    ".gitignore",
    "colcon_defaults.yaml",
)


def _tracked_workspace_files(workspace_root: Path) -> list[Path]:
    completed = subprocess.run(
        [
            "git",
            "-C",
            str(workspace_root),
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    files: list[Path] = []
    for raw in completed.stdout.split(b"\x00"):
        if not raw:
            continue
        relative = Path(raw.decode("utf-8"))
        candidate = workspace_root / relative
        if candidate.is_file():
            files.append(relative)
    return files


def _source_files(package_root: Path, workspace_root: Path) -> list[tuple[Path, str]]:
    try:
        package_prefix = package_root.relative_to(workspace_root)
    except ValueError as exc:
        raise RuntimeError(f"package root {package_root} must live under workspace root {workspace_root}") from exc

    archive_files: dict[str, Path] = {}
    for relative in _tracked_workspace_files(workspace_root):
        archive_relative: str | None = None
        if relative.as_posix() in WORKSPACE_SIDECARS:
            archive_relative = relative.name
        else:
            try:
                archive_relative = relative.relative_to(package_prefix).as_posix()
            except ValueError:
                continue
            if archive_relative in WORKSPACE_SIDECARS:
                continue

        candidate = workspace_root / relative
        if candidate.is_file():
            archive_files[archive_relative] = candidate

    return sorted(((source_path, archive_relative) for archive_relative, source_path in archive_files.items()),
                  key=lambda item: item[1])


def _verify_archive(package_root: Path, workspace_root: Path, archive_path: Path) -> None:
    verifier = Path(__file__).with_name("verify_source_archive.py")
    subprocess.run(
        [
            sys.executable,
            str(verifier),
            "--package-root",
            str(package_root),
            "--workspace-root",
            str(workspace_root),
            "--archive",
            str(archive_path),
        ],
        check=True,
    )


def _write_archive_sidecars(archive_path: Path, source_files: list[str]) -> tuple[Path, Path]:
    manifest_path = archive_path.with_suffix(archive_path.suffix + ".manifest.txt")
    sha256_path = archive_path.with_suffix(archive_path.suffix + ".sha256")

    normalized_files = sorted(source_files)
    archive_hash = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    manifest_lines = [
        f"archive={archive_path.name}",
        f"sha256={archive_hash}",
        f"file_count={len(normalized_files)}",
        "files:",
    ]
    manifest_lines.extend(f"  {relative_path}" for relative_path in normalized_files)
    manifest_path.write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")
    sha256_path.write_text(f"{archive_hash}  {archive_path.name}\n", encoding="utf-8")
    return manifest_path, sha256_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-root", required=True)
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--archive", required=True)
    args = parser.parse_args()

    package_root = Path(args.package_root).resolve()
    workspace_root = Path(args.workspace_root).resolve()
    archive_path = Path(args.archive).resolve()
    archive_path.parent.mkdir(parents=True, exist_ok=True)

    if not package_root.is_dir():
        raise FileNotFoundError(f"package root does not exist: {package_root}")
    if not workspace_root.is_dir():
        raise FileNotFoundError(f"workspace root does not exist: {workspace_root}")

    source_files = _source_files(package_root, workspace_root)
    if not source_files:
        raise RuntimeError(f"no source files found for package root {package_root}")

    if archive_path.exists():
        archive_path.unlink()

    with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for source_path, archive_relative in source_files:
            archive.write(source_path, archive_relative)

    _verify_archive(package_root, workspace_root, archive_path)
    manifest_path, sha256_path = _write_archive_sidecars(
        archive_path, [archive_relative for _, archive_relative in source_files]
    )
    print(f"verified source archive created: {archive_path}")
    print(f"archive manifest written: {manifest_path}")
    print(f"archive sha256 written: {sha256_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
