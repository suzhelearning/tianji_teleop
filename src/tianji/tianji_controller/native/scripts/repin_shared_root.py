#!/usr/bin/env python3
"""Re-pin the shared-root content addresses to the files on disk.

The input contract, the robot geometry artifacts, and the source-evidence
manifest are content-addressed: editing any of them invalidates the digests that
identify them. Run this after such an edit, then re-run the native suite.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[1] / "config"
OPTIONS = Path(__file__).resolve().parents[1] / "src" / "shared_root_options.cpp"
INPUT = "shared_root_tjvr_input_contract.yaml"
GEOMETRY = ("shared_root_robot_geometry.yaml", "shared_root_robot_geometry_ceres.yaml")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="report drift without writing")
    args = parser.parse_args()

    input_sha = digest(CONFIG / INPUT)
    geometry = {name: digest(CONFIG / name) for name in GEOMETRY}
    source = OPTIONS.read_text()

    recorded = set(re.findall(r'"([0-9a-f]{64})"', source))
    stale = [sha for sha in geometry.values() if sha not in recorded]
    changed = False

    for name in GEOMETRY:
        path = CONFIG / name
        text = path.read_text()
        updated = re.sub(r"tjvr_input_contract_sha256: [0-9a-f]{64}",
                         f"tjvr_input_contract_sha256: {input_sha}", text)
        if updated != text:
            changed = True
            if not args.check:
                path.write_text(updated)

    if changed:
        geometry = {name: digest(CONFIG / name) for name in GEOMETRY}

    allow = " ||\n      ".join(f'out.geometry_sha256=="{sha}"' for sha in geometry.values())
    updated = re.sub(r'require\(out\.geometry_sha256=="[0-9a-f]{64}".*?,"unsupported robot geometry revision"\)',
                     f'require({allow},\n      "unsupported robot geometry revision")',
                     source, flags=re.S)
    if args.check:
        print(f"input contract {input_sha}")
        for name, sha in geometry.items():
            print(f"{'STALE' if sha in stale else 'ok   '} {name} {sha}")
        return 1 if stale or updated != source or changed else 0

    if updated != source:
        OPTIONS.write_text(updated)
    print(f"input contract {input_sha}")
    for name, sha in geometry.items():
        print(f"geometry {name} {sha}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
