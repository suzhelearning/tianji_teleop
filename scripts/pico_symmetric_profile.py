"""Offline derived geometry; relative pointer survives named-draft publication."""
import argparse
import json
import os
from pathlib import Path
import sys
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tracking/src/pico_bridge/scripts'))
from pico_symmetric_geometry import FILES, POLICY_FILE, create_profile, digest

POINTER = 'runtime_symmetric.json'


def generate(source):
    source = Path(source).resolve()
    if (source / POLICY_FILE).exists():
        raise ValueError('cannot calibrate or derive inside a symmetric snapshot')
    if not all((source / name).is_file() for name in FILES):
        print('Symmetric geometry pending: both complete calibration chains required', file=sys.stderr)
        return None
    snapshots = source / 'symmetric_profiles'
    if snapshots.is_symlink():
        raise ValueError('snapshot directory cannot be a symlink')
    relative = Path('symmetric_profiles') / ('sym-' + uuid4().hex)
    result = source / relative
    create_profile(source, result)
    # Only publish a fully validated snapshot; keep old pointer on failure.
    temp = source / ('.symmetric-' + uuid4().hex)
    with temp.open('x') as stream:
        json.dump({'schema_version': 1, 'relative_path': str(relative)}, stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, source / POINTER)
    print(f'Symmetric geometry generated: {result}', file=sys.stderr)
    return result


def resolve(source):
    source = Path(source).resolve()
    pointer = source / POINTER
    if pointer.is_symlink():
        raise ValueError('symmetric pointer cannot be a symlink')
    doc = json.loads(pointer.read_text())
    relative = Path(doc['relative_path'])
    if doc.get('schema_version') != 1 or len(relative.parts) != 2 or relative.parts[0] != 'symmetric_profiles' or not relative.parts[1].startswith('sym-'):
        raise ValueError('invalid symmetric pointer')
    result = source / relative
    if result.resolve() != result or not result.is_dir():
        raise ValueError('invalid symmetric snapshot path')
    policy = json.loads((result / POLICY_FILE).read_text())
    if policy.get('complete') is not True or policy.get('schema_version') != 1 or policy.get('policy') != 'symmetric_max':
        raise ValueError('incomplete symmetric snapshot')
    for name in FILES:
        expected = policy.get('source_sha256', {}).get(name)
        if digest(source / name) != expected or digest(result / name) != expected:
            raise ValueError('stale symmetric snapshot: calibration changed: ' + name)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--resolve', action='store_true')
    args = parser.parse_args()
    try:
        result = resolve(args.source) if args.resolve else generate(args.source)
        if result is not None:
            print(result)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, str(error) + '\n')


if __name__ == '__main__':
    main()
