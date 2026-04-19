#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    root = Path(args.root).resolve()
    manifest = json.loads((root / 'docs' / 'reference' / 'xmate6_official_alignment_manifest.json').read_text(encoding='utf-8'))
    payload = {
        'canonical_identity': manifest.get('identity', {}).get('canonical_identity', 'unknown'),
        'canonical_package': manifest.get('identity', {}).get('canonical_package', 'unknown'),
        'legacy_source_package': manifest.get('identity', {}).get('legacy_source_package', 'unknown'),
        'unsupported_public_modules': manifest.get('unsupported_public_modules', []),
        'public_examples': manifest.get('public_examples', []),
        'internal_examples': manifest.get('internal_examples', []),
        'public_motion_extensions': manifest.get('public_motion_extensions', []),
        'profiles': manifest.get('profiles', {}),
        'source_layout': manifest.get('source_layout', {}),
        'release_gates': manifest.get('release_gates', []),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
