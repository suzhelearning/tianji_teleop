#!/usr/bin/env python3
"""Resolve or calibrate a named person's pinned teleoperation calibration."""
from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import re
import shutil
import signal
import stat
import subprocess
import sys
import time
from uuid import uuid4

import yaml


ROOT = Path(__file__).resolve().parent
PICO_SCRIPTS = ROOT / "tracking/src/pico_bridge/scripts"
_COMPONENT_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*")
_INTERRUPT_GRACE_SECONDS = 5.0
_PICO_ARTIFACTS = tuple(
    f"pico_{side}_{kind}.yaml"
    for side in ("left", "right")
    for kind in ("palm_tcp", "wrist_pivot", "arm_geometry")
)


def _name(value: object, label: str) -> str:
    if not isinstance(value, str) or _COMPONENT_NAME.fullmatch(value) is None:
        raise ValueError(f"{label} must be a single name using letters, digits, '_' or '-'")
    return value


def _contained(path: Path, boundary: Path, *, directory: bool = False) -> Path:
    try:
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ValueError(f"missing or inaccessible path: {path}") from error
    if not resolved.is_relative_to(boundary):
        raise ValueError(f"path escapes selected profile or calibration: {path}")
    if not (resolved.is_dir() if directory else resolved.is_file()):
        raise ValueError(f"expected {'directory' if directory else 'file'}: {path}")
    return resolved


def _load_profile(user: str, root: Path) -> tuple[Path, dict]:
    user = _name(user, "user")
    profiles = _contained(root / "profiles", root, directory=True)
    # A person's directory cannot be an alias for another person's profile.
    person_path = profiles / user
    person = _contained(person_path, person_path, directory=True)
    path = _contained(person / "profile.yaml", person)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read profile {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"profile must be a mapping: {path}")
    if set(document) - {"schema_version", "user_id", "pico", "manus"}:
        raise ValueError(f"unknown profile fields: {path}")
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise ValueError(f"profile schema_version must be integer 1: {path}")
    if document.get("user_id") != user:
        raise ValueError(f"profile user_id must match selected user {user!r}: {path}")
    return person, document


def available_users(root: Path = ROOT) -> list[str]:
    """List valid profile identities without inspecting component artifacts."""
    root = Path(root).resolve()
    profiles = root / "profiles"
    if not profiles.is_dir():
        return []
    users = []
    for path in profiles.iterdir():
        try:
            _load_profile(path.name, root)
        except ValueError:
            continue
        users.append(path.name)
    return sorted(users)


def resolve_profile(user: str, component: str, root: Path = ROOT) -> str:
    """Return a validated PICO revision directory or Manus calibration user.

    Only the requested component is inspected. Missing or invalid selections
    raise ValueError; no global calibration or alternate revision is consulted.
    """
    if component not in {"pico", "manus"}:
        raise ValueError(f"unknown component: {component}")
    root = Path(root).resolve()
    person, profile = _load_profile(user, root)
    selection = profile.get(component)
    field = "active_set" if component == "pico" else "user"
    if not isinstance(selection, dict) or set(selection) != {field}:
        raise ValueError(f"profile {user!r} {component} must contain only {field!r}")
    selected = _name(selection[field], f"{component}.{field}")
    if component == "manus":
        calibration_path = root / "manus" / "calibration"
        calibration = _contained(calibration_path, calibration_path, directory=True)
        for side in ("Left", "Right"):
            path = calibration / f"{selected}{side}MetaglovePro.mcal"
            _contained(path, path)
        return selected

    pico_path = person / "pico"
    pico = _contained(pico_path, pico_path, directory=True)
    revision_path = pico / selected
    revision = _contained(revision_path, revision_path, directory=True)
    _validate_pico_revision(revision)
    return str(revision)


def _validate_pico_revision(revision: Path) -> None:
    # Import source validation lazily: listing and Manus do not need NumPy or PICO.
    if str(PICO_SCRIPTS) not in sys.path:
        sys.path.insert(0, str(PICO_SCRIPTS))
    from pico_calibration_artifact import validate_artifact

    for side in ("left", "right"):
        artifacts = {
            kind: _contained(revision / f"pico_{side}_{kind}.yaml", revision)
            for kind in ("palm_tcp", "wrist_pivot", "arm_geometry")
        }
        try:
            validate_artifact(
                artifacts["arm_geometry"],
                "geometry",
                side,
                tcp_path=artifacts["palm_tcp"],
                wrist_path=artifacts["wrist_pivot"],
            )
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            raise ValueError(f"invalid {side} PICO calibration in {revision}: {error}") from error


def _writable_directory(path: Path) -> Path:
    if path.is_symlink():
        raise ValueError(f"writable calibration directory cannot be a symlink: {path}")
    path.mkdir(exist_ok=True)
    return _contained(path, path, directory=True)


def _check_writable_tree(directory: Path) -> None:
    """Never let calibration writes follow aliases into published or external data."""
    _contained(directory, directory, directory=True)
    for entry in directory.iterdir():
        info = entry.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise ValueError(f"calibration data cannot contain symlinks: {entry}")
        if stat.S_ISDIR(info.st_mode):
            _check_writable_tree(entry)
        elif not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError(f"calibration data must contain unaliased regular files: {entry}")


def _atomic_text(path: Path, content: str) -> None:
    temporary = path.parent / f".{path.name}-{uuid4().hex}.tmp"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def _recover_publication(person: Path, pico: Path, user: str, root: Path) -> None:
    """Recover a rename interrupted before profile replacement, without changing a pin."""
    journal = person / ".pico-publication"
    if not journal.exists() and not journal.is_symlink():
        return
    if journal.is_symlink() or journal.stat().st_nlink != 1:
        raise ValueError(f"invalid publication journal: {journal}")
    revision_name = _name(journal.read_text(encoding="utf-8").strip(), "pending revision")
    revision = pico / revision_name
    draft = pico / ".draft"
    profile = {}
    if (person / "profile.yaml").exists() or (person / "profile.yaml").is_symlink():
        _, profile = _load_profile(user, root)
    selection = profile.get("pico")
    if isinstance(selection, dict) and selection.get("active_set") == revision_name:
        # Replacement committed; this directory is already an immutable revision.
        _contained(revision, revision, directory=True)
        if draft.exists() or draft.is_symlink():
            raise ValueError("published calibration unexpectedly still has a draft")
    elif revision.exists() or revision.is_symlink():
        if draft.exists() or draft.is_symlink():
            raise ValueError("interrupted publication has both a draft and a revision")
        _check_writable_tree(revision)
        revision.rename(draft)
    elif not draft.is_dir():
        raise ValueError("interrupted publication has neither its draft nor revision")
    journal.unlink()


def _run_calibration(command: list[str], environment: dict[str, str]) -> int:
    child = None
    interrupted = 0
    interrupt_deadline = None

    def forward(signum, _frame):
        nonlocal interrupted, interrupt_deadline
        if not interrupted:
            interrupted = signum
            interrupt_deadline = time.monotonic() + _INTERRUPT_GRACE_SECONDS
        if child is not None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    try:
        for signum in previous:
            signal.signal(signum, forward)
        # Inherit terminal descriptors, but own the whole calibration process group.
        child = subprocess.Popen(command, env=environment, start_new_session=True)
        if interrupted:
            forward(interrupted, None)
        while True:
            try:
                result = child.wait(timeout=0.1)
                break
            except subprocess.TimeoutExpired:
                if interrupt_deadline is not None and time.monotonic() >= interrupt_deadline:
                    try:
                        os.killpg(child.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    result = child.wait()
                    break
        if interrupted:
            return 128 + interrupted
        return result if result >= 0 else 128 - result
    finally:
        if child is not None:
            # The shell leader may exit before descendant traps stop their
            # setsid publishers. Give those traps the remaining bounded grace.
            deadline = interrupt_deadline or (time.monotonic() + _INTERRUPT_GRACE_SECONDS)
            try:
                if not interrupted:
                    os.killpg(child.pid, signal.SIGTERM)
                while time.monotonic() < deadline:
                    os.killpg(child.pid, 0)
                    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def calibrate_profile(user: str, command: list[str], root: Path = ROOT) -> int:
    """Run a PICO calibration in a locked persistent draft and publish valid chains.

    An incomplete draft is successful progress, not a usable profile. Child failures
    retain progress and their exit status. No global calibration is ever seeded.
    """
    user = _name(user, "user")
    if not command or any(not isinstance(argument, str) for argument in command) or not command[0]:
        raise ValueError("calibration requires a command and arguments")
    root = Path(root).resolve()
    profiles = _writable_directory(root / "profiles")
    person = _writable_directory(profiles / user)
    lock_path = person / ".calibration.lock"
    try:
        descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    except OSError as error:
        raise ValueError(f"cannot open calibration lock: {lock_path}") from error
    with os.fdopen(descriptor, "r+") as lock:
        info = os.fstat(lock.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError(f"invalid calibration lock: {lock_path}")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError(f"calibration already running for user {user!r}") from error
        pico = _writable_directory(person / "pico")
        _recover_publication(person, pico, user, root)
        profile_path = person / "profile.yaml"
        profile = {"schema_version": 1, "user_id": user, "manus": {"user": user}}
        active = None
        if profile_path.exists() or profile_path.is_symlink():
            _, profile = _load_profile(user, root)
            if "pico" in profile:
                active = Path(resolve_profile(user, "pico", root=root))
        draft = pico / ".draft"
        if draft.exists() or draft.is_symlink():
            _check_writable_tree(draft)
        else:
            # Seed in a private staging directory so interrupted copying is not a draft.
            seed = pico / f".seed-{uuid4().hex}"
            seed.mkdir()
            try:
                if active is not None:
                    for filename in _PICO_ARTIFACTS:
                        shutil.copyfile(active / filename, seed / filename)
                seed.rename(draft)
            finally:
                if seed.exists():
                    shutil.rmtree(seed)
        recordings = _writable_directory(person / "recordings")
        _check_writable_tree(recordings)
        environment = os.environ.copy()
        environment.pop("PICO_CALIBRATION_DIR", None)
        environment["PICO_CALIBRATION_DIR"] = str(draft)
        # Keep source_recording paths stable when the draft becomes a revision.
        environment["PICO_CALIBRATION_RECORDINGS_DIR"] = str(recordings)
        print(f"Calibrating profile user={user}; draft={draft}", file=sys.stderr)
        result = _run_calibration(command, environment)
        if result:
            print(f"Calibration failed ({result}); retained draft={draft}", file=sys.stderr)
            return result
        _check_writable_tree(draft)
        _check_writable_tree(recordings)
        try:
            _validate_pico_revision(draft)
        except ValueError as error:
            print(f"Calibration pending for user={user}; retained draft={draft}: {error}", file=sys.stderr)
            return 0
        revision_name = f"cal-{uuid4().hex}"
        revision = pico / revision_name
        updated = dict(profile)
        updated["pico"] = {"active_set": revision_name}
        journal = person / ".pico-publication"
        _atomic_text(journal, revision_name + "\n")
        try:
            draft.rename(revision)
            _atomic_text(profile_path, yaml.safe_dump(updated, sort_keys=False))
        except BaseException:
            # If replacement committed before interruption, recovery keeps that pin.
            _recover_publication(person, pico, user, root)
            raise
        journal.unlink()
        print(f"Published profile user={user}; PICO revision={revision_name} directory={revision}", file=sys.stderr)
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--user", help="person profile name (see --list-users)")
    parser.add_argument("--component", choices=("pico", "manus"))
    parser.add_argument("--list-users", action="store_true")
    parser.add_argument("--calibrate", action="store_true", help="run a command in this person's PICO draft")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="-- COMMAND [ARGS...] for --calibrate")
    args = parser.parse_args()
    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if args.list_users and (args.user is not None or args.component is not None or args.calibrate or args.command):
        parser.error("--list-users cannot be combined with another mode")
    if args.calibrate and args.component is not None:
        parser.error("--calibrate cannot be combined with --component")
    if args.command and not args.calibrate:
        parser.error("a command requires --calibrate")
    if args.list_users:
        print("\n".join(available_users()))
        return 0
    if args.user is None:
        parser.error("--user is required; available users: " + (", ".join(available_users()) or "(none)"))
    if args.calibrate:
        if not args.command:
            parser.error("--calibrate requires -- COMMAND [ARGS...]")
        try:
            return calibrate_profile(args.user, args.command)
        except (ValueError, OSError, ImportError) as error:
            parser.error(str(error))
    if args.component is None:
        parser.error("--component is required")
    try:
        resolved = resolve_profile(args.user, args.component)
    except (ValueError, OSError, ImportError) as error:
        parser.error(f"{error}; available users: {', '.join(available_users()) or '(none)'}")
    if args.component == "pico":
        selection = f"PICO revision={Path(resolved).name} directory={resolved}"
    else:
        selection = f"Manus calibration user={resolved}"
    print(f"Selected profile user={args.user}; {selection}", file=sys.stderr)
    print(resolved)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
