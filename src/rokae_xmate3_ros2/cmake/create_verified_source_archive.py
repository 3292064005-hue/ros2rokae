#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime
import hashlib
import os
import subprocess
import sys
import zipfile
from pathlib import Path
import xml.etree.ElementTree as ET


WORKSPACE_SIDECARS = (
    ".gitignore",
    "colcon_defaults.yaml",
)


def _enforce_clean_env_if_requested() -> None:
    if os.environ.get("ROKAE_ENFORCE_CLEAN_ENV") != "1":
        return
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix and os.environ.get("ROKAE_CLEAN_ENV_ACTIVE") != "1":
        raise RuntimeError(
            "release archive packaging must run from clean_build_env.sh; "
            f"active Conda environment detected at {conda_prefix}"
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


def _read_package_version(package_root: Path) -> str:
    package_xml = package_root / "package.xml"
    if not package_xml.is_file():
        raise FileNotFoundError(f"package.xml does not exist: {package_xml}")
    root = ET.parse(package_xml).getroot()
    version = ""
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "version":
            continue
        version = (element.text or "").strip()
        if version:
            break
    if not version:
        raise RuntimeError(f"package.xml is missing a <version> entry: {package_xml}")
    return version


def _git_commit(workspace_root: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(workspace_root), "rev-parse", "HEAD"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        return "unknown"
    return completed.stdout.strip() or "unknown"


def _write_archive_sidecars(archive_path: Path,
                            source_files: list[str],
                            package_version: str,
                            git_commit: str,
                            generated_at_utc: str) -> tuple[Path, Path]:
    manifest_path = archive_path.with_suffix(archive_path.suffix + ".manifest.txt")
    sha256_path = archive_path.with_suffix(archive_path.suffix + ".sha256")

    normalized_files = sorted(source_files)
    archive_hash = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    manifest_lines = [
        f"archive={archive_path.name}",
        f"sha256={archive_hash}",
        "archive_root_mode=package_root+whitelist_sidecars",
        f"package_version={package_version}",
        f"git_commit={git_commit}",
        f"generated_at_utc={generated_at_utc}",
        "source_tree_kind=verified_release_candidate",
        f"file_count={len(normalized_files)}",
        "files:",
    ]
    manifest_lines.extend(f"  {relative_path}" for relative_path in normalized_files)
    manifest_path.write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")
    sha256_path.write_text(f"{archive_hash}  {archive_path.name}\n", encoding="utf-8")
    return manifest_path, sha256_path


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
    archive_path.parent.mkdir(parents=True, exist_ok=True)

    if not package_root.is_dir():
        raise FileNotFoundError(f"package root does not exist: {package_root}")
    if not workspace_root.is_dir():
        raise FileNotFoundError(f"workspace root does not exist: {workspace_root}")

    source_files = _source_files(package_root, workspace_root)
    if not source_files:
        raise RuntimeError(f"no source files found for package root {package_root}")
    package_version = _read_package_version(package_root)
    git_commit = _git_commit(workspace_root)
    generated_at_utc = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0).isoformat()

    if archive_path.exists():
        archive_path.unlink()

    with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for source_path, archive_relative in source_files:
            archive.write(source_path, archive_relative)

    _verify_archive(package_root, workspace_root, archive_path)
    manifest_path, sha256_path = _write_archive_sidecars(
        archive_path,
        [archive_relative for _, archive_relative in source_files],
        package_version,
        git_commit,
        generated_at_utc,
    )
    print(f"verified source archive created: {archive_path}")
    print(f"archive manifest written: {manifest_path}")
    print(f"archive sha256 written: {sha256_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
