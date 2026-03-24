#!/usr/bin/env python3

from __future__ import annotations

import argparse
import subprocess
import sys
import zipfile
from pathlib import Path


def _source_files(source_root: Path) -> list[Path]:
    completed = subprocess.run(
        [
            "git",
            "-C",
            str(source_root),
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
        candidate = source_root / relative
        if candidate.is_file():
            files.append(relative)
    return files


def _verify_archive(source_root: Path, archive_path: Path) -> None:
    verifier = Path(__file__).with_name("verify_source_archive.py")
    subprocess.run(
        [sys.executable, str(verifier), "--source-root", str(source_root), "--archive", str(archive_path)],
        check=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--archive", required=True)
    args = parser.parse_args()

    source_root = Path(args.source_root).resolve()
    archive_path = Path(args.archive).resolve()
    archive_path.parent.mkdir(parents=True, exist_ok=True)

    source_files = _source_files(source_root)
    if not source_files:
        raise RuntimeError(f"no source files found under {source_root}")

    if archive_path.exists():
        archive_path.unlink()

    with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for relative_path in source_files:
            archive.write(source_root / relative_path, relative_path.as_posix())

    _verify_archive(source_root, archive_path)
    print(f"verified source archive created: {archive_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
