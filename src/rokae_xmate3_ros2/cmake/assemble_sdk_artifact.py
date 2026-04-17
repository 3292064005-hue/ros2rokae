#!/usr/bin/env python3
"""Assemble a deterministic SDK artifact directory from an install staging prefix."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


def sha256_file(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as fh:
        while True:
            block = fh.read(1024 * 1024)
            if not block:
                break
            hasher.update(block)
    return hasher.hexdigest()


def copy_required(src: Path, dst: Path, required: bool = True) -> bool:
    if not src.exists():
        if required:
            raise FileNotFoundError(f"required artifact path missing: {src}")
        return False
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=True)
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--staging-prefix", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    staging_prefix = Path(args.staging_prefix).resolve()
    output_dir = Path(args.output_dir).resolve()

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    copy_required(staging_prefix / "include" / "rokae", output_dir / "include" / "rokae")
    copy_required(staging_prefix / "lib" / "cmake" / "xCoreSDK", output_dir / "lib" / "cmake" / "xCoreSDK")
    copy_required(staging_prefix / "bin" / "rokae_sim_runtime", output_dir / "bin" / "rokae_sim_runtime", required=False)

    for name in ("libxCoreSDK_static.a", "libxCoreSDK_shared.so"):
        copy_required(staging_prefix / "lib" / name, output_dir / "lib" / name, required=False)

    for rel in [
        (Path('share/rokae_xmate3_ros2/docs'), Path('share/rokae_xmate3_ros2/docs')),
        (Path('share/rokae_xmate3_ros2/examples'), Path('share/rokae_xmate3_ros2/examples')),
        (Path('share/rokae_xmate3_ros2/launch'), Path('share/rokae_xmate3_ros2/launch')),
        (Path('share/rokae_xmate3_ros2/config'), Path('share/rokae_xmate3_ros2/config')),
        (Path('share/rokae_xmate3_ros2/generated'), Path('share/rokae_xmate3_ros2/generated')),
        (Path('share/rokae_xmate3_ros2/meshes'), Path('share/rokae_xmate3_ros2/meshes')),
        (Path('share/rokae_xmate3_ros2/models'), Path('share/rokae_xmate3_ros2/models')),
        (Path('share/rokae_xmate3_ros2/tools'), Path('share/rokae_xmate3_ros2/tools')),
        (Path('share/rokae_xmate3_ros2/urdf'), Path('share/rokae_xmate3_ros2/urdf')),
        (Path('share/rokae_xmate3_ros2/worlds'), Path('share/rokae_xmate3_ros2/worlds')),
    ]:
        copy_required(staging_prefix / rel[0], output_dir / rel[1], required=False)

    entries = []
    for path in sorted(output_dir.rglob("*")):
        if path.is_file():
            rel = path.relative_to(output_dir).as_posix()
            entries.append(
                {
                    "path": rel,
                    "size": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )

    manifest = {
        "artifact_kind": "xcoresdk_public_xmate6",
        "target_family": "xmate6",
        "source": str(staging_prefix),
        "file_count": len(entries),
        "files": entries,
    }
    manifest_path = output_dir / "artifact_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"assembled SDK artifact: {output_dir}")
    print(f"manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
