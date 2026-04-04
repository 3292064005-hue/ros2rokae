#!/usr/bin/env python3
"""Check that the shared compatibility library does not leak backend/internal symbol names.

The check must examine the dynamic/exported symbol table only. Using the full symbol table would
flag local implementation symbols and produce false positives, which would make the ABI gate too
strict for shared-library installs.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

FORBIDDEN = (
    'rokae::ros2::',
    'sdk_shim',
    'RobotSession',
)


def read_dynamic_symbols(library: Path) -> str:
    """Return the demangled exported/dynamic symbol table for *library*.

    Args:
        library: Shared library to inspect.

    Returns:
        A text dump of the exported/dynamic symbols.

    Raises:
        RuntimeError: If neither nm nor readelf is available, or the underlying command fails.
    """
    if shutil.which('nm'):
        try:
            result = subprocess.run(
                ['nm', '-D', '-C', str(library)], check=True, capture_output=True, text=True
            )
            return result.stdout
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f'nm failed for {library}: {exc.stderr or exc.stdout}') from exc

    if shutil.which('readelf'):
        try:
            result = subprocess.run(
                ['readelf', '--dyn-syms', '--wide', '--demangle', str(library)],
                check=True,
                capture_output=True,
                text=True,
            )
            return result.stdout
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f'readelf failed for {library}: {exc.stderr or exc.stdout}') from exc

    raise RuntimeError('neither nm nor readelf is available; cannot inspect exported symbols')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--library', required=True)
    args = parser.parse_args()

    library = Path(args.library)
    if not library.exists():
        print(f'missing shared library: {library}', file=sys.stderr)
        return 1

    try:
        symbol_dump = read_dynamic_symbols(library)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    leaked = [needle for needle in FORBIDDEN if needle in symbol_dump]
    if leaked:
        print('compat exported symbol check failed:')
        for needle in leaked:
            print(f'- forbidden symbol fragment found in exported symbol table: {needle}')
        return 1

    print('compat exported symbol check passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
