#!/usr/bin/env python3
"""Verify install-tree canonical robot-description rendering falls back to installed xacro when shape args differ."""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import stat
import subprocess
import sys
import tempfile


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RENDERER = REPO_ROOT / 'tools' / 'render_robot_description.py'


def write_text(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def run_renderer(package_share: pathlib.Path,
                 canonical_urdf: pathlib.Path,
                 metadata: pathlib.Path,
                 *,
                 backend_mode: str = 'jtc',
                 service_exposure_profile: str = 'public_xmate6_only',
                 enable_xcore_plugin: str = 'true',
                 enable_ros2_control: str = 'true') -> str:
    cmd = [
        sys.executable,
        str(RENDERER),
        '--model', str(canonical_urdf),
        '--package-share', str(package_share),
        '--mesh-root', 'model://rokae_xmate3_ros2/meshes/',
        '--enable-ros2-control', enable_ros2_control,
        '--enable-xcore-plugin', enable_xcore_plugin,
        '--backend-mode', backend_mode,
        '--service-exposure-profile', service_exposure_profile,
        '--canonical-model', str(canonical_urdf),
        '--canonical-metadata', str(metadata),
    ]
    completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return completed.stdout


def main() -> int:
    if not RENDERER.exists():
        raise FileNotFoundError(f'renderer not found: {RENDERER}')

    with tempfile.TemporaryDirectory(prefix='rokae_render_install_tree_') as tmpdir:
        root = pathlib.Path(tmpdir)
        package_share = root / 'share' / 'rokae_xmate3_ros2'
        canonical_urdf = package_share / 'generated' / 'urdf' / 'xMate3.urdf'
        metadata = package_share / 'generated' / 'urdf' / 'xMate3.description.json'
        installed_xacro = package_share / 'urdf' / 'xMate3.xacro'
        fake_bin = root / 'bin'
        fake_xacro = fake_bin / 'xacro'

        write_text(canonical_urdf, '<robot name="canonical" backend="jtc" profile="public_xmate6_only" plugin="true" control="true"/>\n')
        write_text(installed_xacro, '<!-- fake install-tree xacro source -->\n')
        metadata_payload = {
            'schema_version': 2,
            'source_xacro': str(installed_xacro),
            'source_xacro_package_relative': 'urdf/xMate3.xacro',
            'xacro_args': {
                'enable_ros2_control': 'true',
                'enable_xcore_plugin': 'true',
                'backend_mode': 'jtc',
                'service_exposure_profile': 'public_xmate6_only',
            },
        }
        write_text(metadata, json.dumps(metadata_payload))

        fake_xacro_script = '''#!/usr/bin/env python3
import sys
from pathlib import Path
pairs = {}
for item in sys.argv[2:]:
    if ':=' in item:
        key, value = item.split(':=', 1)
        pairs[key] = value
print('<robot source="{}" backend="{}" profile="{}" plugin="{}" control="{}"/>'.format(
    Path(sys.argv[1]).name,
    pairs.get('backend_mode', ''),
    pairs.get('service_exposure_profile', ''),
    pairs.get('enable_xcore_plugin', ''),
    pairs.get('enable_ros2_control', '')))
'''
        write_text(fake_xacro, fake_xacro_script)
        fake_xacro.chmod(fake_xacro.stat().st_mode | stat.S_IEXEC)

        env = os.environ.copy()
        env['PATH'] = f"{fake_bin}{os.pathsep}{env.get('PATH', '')}"

        def invoke(**kwargs: str) -> str:
            cmd = [
                sys.executable,
                str(RENDERER),
                '--model', str(canonical_urdf),
                '--package-share', str(package_share),
                '--mesh-root', 'model://rokae_xmate3_ros2/meshes/',
                '--enable-ros2-control', kwargs.get('enable_ros2_control', 'true'),
                '--enable-xcore-plugin', kwargs.get('enable_xcore_plugin', 'true'),
                '--backend-mode', kwargs.get('backend_mode', 'jtc'),
                '--service-exposure-profile', kwargs.get('service_exposure_profile', 'public_xmate6_only'),
                '--canonical-model', str(canonical_urdf),
                '--canonical-metadata', str(metadata),
            ]
            completed = subprocess.run(cmd, check=True, capture_output=True, text=True, env=env)
            return completed.stdout.strip()

        canonical_out = invoke()
        assert canonical_out == '<robot name="canonical" backend="jtc" profile="public_xmate6_only" plugin="true" control="true"/>', canonical_out

        internal_out = invoke(service_exposure_profile='internal_full')
        assert internal_out == '<robot source="xMate3.xacro" backend="jtc" profile="internal_full" plugin="true" control="true"/>', internal_out

        hybrid_out = invoke(backend_mode='hybrid', service_exposure_profile='internal_full')
        assert hybrid_out == '<robot source="xMate3.xacro" backend="hybrid" profile="internal_full" plugin="true" control="true"/>', hybrid_out

        plugin_off_out = invoke(enable_xcore_plugin='false')
        assert plugin_off_out == '<robot source="xMate3.xacro" backend="jtc" profile="public_xmate6_only" plugin="false" control="true"/>', plugin_off_out

        control_off_out = invoke(enable_ros2_control='false')
        assert control_off_out == '<robot source="xMate3.xacro" backend="jtc" profile="public_xmate6_only" plugin="true" control="false"/>', control_off_out

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
