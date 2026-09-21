#!/usr/bin/env python3
"""Named left-only calibration lifecycle. No changes to legacy profile pins."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
from uuid import uuid4

import yaml

from tianji_runtime.resources import workspace

ROOT = workspace()
from teleop_profile import _name, _writable_directory, _check_writable_tree, _atomic_text, _run_calibration, _validate_pico_revision
from pico_simple_artifact import MANIFEST, LEFT_TCP, LEFT_WRIST, digest, payloads, height_lengths


def resolve(user, root=ROOT):
    user = _name(user, "user")
    root = Path(root).resolve()
    base = root / "profiles" / user / "pico-simple"
    for path in (root / "profiles", base.parent, base):
        if path.is_symlink():
            raise ValueError("simple profile directory cannot be a symlink")
    _check_writable_tree(base)
    selected = json.loads((base / "active.json").read_text())
    if not isinstance(selected, dict) or set(selected) != {"schema_version", "active_set"} or type(selected["schema_version"]) is not int or selected["schema_version"] != 1:
        raise ValueError("invalid simple profile pointer")
    name = _name(selected.get("active_set"), "simple active_set")
    if not name.startswith("cal-"):
        raise ValueError("invalid simple revision name")
    revision = base / name
    _check_writable_tree(revision)
    if not (revision / MANIFEST).is_file():
        raise ValueError("simple manifest missing")
    _validate_pico_revision(revision)
    return revision.resolve()


def build_bundle(revision, height, *, height_wrist=False):
    height_lengths(height)
    revision = Path(revision)
    sources = (LEFT_TCP,) if height_wrist else (LEFT_TCP, LEFT_WRIST)
    manifest = dict(schema_version=2 if height_wrist else 1, model="symmetric_local_y", height_m=height,
        accepted_model_assumption=True, hardware_acceptance_complete=False,
        source_sha256={n: digest(revision / n) for n in sources},
        provenance=("left palm TCP measured; bilateral wrist distance estimated from height; no wrist capture"
                    if height_wrist else "left TCP then left wrist collected by simple lifecycle; no independent right measurement"))
    if height_wrist:
        manifest["wrist_model"] = "height_times_0.037037_palm_local_positive_x"
    documents = payloads(revision, manifest)
    for name, document in documents.items():
        with (revision / name).open("x") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
    with (revision / MANIFEST).open("x") as stream:
        json.dump(manifest, stream, indent=2)
    _validate_pico_revision(revision)


def calibrate(user, height, root=ROOT, runner=_run_calibration, confirm=input, on_publish=None):
    user = _name(user, "user")
    height_lengths(height)  # Before creating directories or launching children.
    root = Path(root).resolve()
    profiles = _writable_directory(root / "profiles")
    person = _writable_directory(profiles / user)
    # Same lock as the original named calibration, but a separate active pointer.
    fd = os.open(person / ".calibration.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    with os.fdopen(fd, "r+") as lock:
        import stat
        info = os.fstat(lock.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError("invalid calibration lock")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        base = _writable_directory(person / "pico-simple")
        name = "cal-" + uuid4().hex
        revision = _writable_directory(base / name)
        recordings = _writable_directory(person / "recordings")
        recordings = _writable_directory(recordings / ("simple-" + name))
        env = os.environ.copy()
        env.update(PICO_CALIBRATION_DIR=str(revision),
                   PICO_CALIBRATION_RECORDINGS_DIR=str(recordings), PICO_GEOMETRY_POLICY="original")
        command = ["bash", str(root / "bash/calibrate_pico_arm.sh"), "left"]
        print(f"Simple calibration user={user}; isolated revision={revision}", flush=True)
        result = runner(command + ["tcp"], env)
        if result:
            return result
        _check_writable_tree(revision)
        build_bundle(revision, height, height_wrist=True)
        print((revision / MANIFEST).read_text(), flush=True)
        for filename in ("pico_right_palm_tcp.yaml", "pico_right_wrist_pivot.yaml", "pico_left_arm_geometry.yaml"):
            print(f"{filename}:\n{(revision / filename).read_text()}", flush=True)
        print("Model: bilateral common lengths; right TCP is Y-mirrored left, not measured. "
              "No robot motion is authorized. Old profile.yaml is unchanged.", flush=True)
        if confirm("Type publish to select this simple profile (anything else keeps current selection): ").strip() != "publish":
            print(f"Not published; files retained: {revision}", flush=True)
            return 0
        _validate_pico_revision(revision)
        _atomic_text(base / "active.json", json.dumps({"schema_version": 1, "active_set": name}) + "\n")
        print(f"Published simple user={user}: {revision}", flush=True)
        if on_publish is not None:
            on_publish(revision.resolve())
    return 0


def available_users(root=ROOT):
    profiles = Path(root) / "profiles"
    if not profiles.is_dir():
        return []
    users = []
    for person in profiles.iterdir():
        try:
            resolve(person.name, root)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
            continue
        users.append(person.name)
    return sorted(users)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user")
    parser.add_argument("--list-users", action="store_true")
    parser.add_argument("--height-m", type=float)
    parser.add_argument("--accept-symmetric-model", action="store_true",
                        help="accept symmetric local-Y axes and mirrored grip as model assumptions")
    parser.add_argument("--resolve", action="store_true", help="read-only resolve of the separate simple profile")
    args = parser.parse_args()
    if args.list_users:
        if args.user or args.resolve or args.height_m is not None or args.accept_symmetric_model:
            parser.error("--list-users cannot take calibration options")
        print("\n".join(available_users()))
        return 0
    if not args.user:
        parser.error("--user is required")
    try:
        if args.resolve:
            if args.height_m is not None or args.accept_symmetric_model:
                parser.error("--resolve cannot take calibration options")
            print(resolve(args.user))
            return 0
        if args.height_m is None or not args.accept_symmetric_model:
            parser.error("calibration requires --height-m and --accept-symmetric-model")
        if not sys.stdin.isatty():
            parser.error("interactive terminal required")
        return calibrate(args.user, args.height_m)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(2, str(error) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
