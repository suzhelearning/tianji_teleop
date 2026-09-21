#!/usr/bin/env python3
"""Select a published input; never switch an existing session's operator."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import re

from tianji_runtime.resources import workspace

ROOT = workspace()
from calibrate_pico_simple import resolve

SESSION = "pico_tianji_teleop"


def release_owner(token, root=ROOT, run=subprocess.run):
    if not re.fullmatch(r'[0-9a-f]{32}', token):
        raise ValueError('invalid simulation owner token')
    present = run(['tmux', 'has-session', '-t', SESSION], capture_output=True, text=True, timeout=5)
    if present.returncode:
        return
    # Evaluate token and kill in tmux, not a check-then-kill by reused session name.
    checkout = run(['tmux', 'show-options', '-t', SESSION, '-v', '@tianji_checkout'],
                   capture_output=True, text=True, timeout=5, check=True).stdout.strip()
    if checkout != str(root.resolve()):
        raise RuntimeError('PICO checkout changed; automatic cleanup refused')
    run(['tmux', 'if-shell', '-F', '-t', SESSION,
         '#{==:#{@tianji_sim_owner},' + token + '}',
         'kill-session -t ' + SESSION, ''], check=True, timeout=5)
    print('PICO cleanup completed: only a session with this simulation owner token may be stopped; reused sessions retained.', flush=True)


def fingerprint(directory):
    directory = Path(directory)
    names = [f"pico_{side}_{kind}.yaml" for side in ("left", "right")
             for kind in ("palm_tcp", "wrist_pivot", "arm_geometry")]
    if (directory / "pico_simple_model.json").exists():
        names.append("pico_simple_model.json")
    digest = hashlib.sha256()
    for name in sorted(names):
        digest.update(name.encode() + b"\0")
        digest.update(hashlib.sha256((directory / name).read_bytes()).digest())
    return digest.hexdigest()


def ensure(user, root=ROOT, run=subprocess.run, resolver=resolve):
    revision = resolver(user, root)
    expected = fingerprint(revision)
    present = run(["tmux", "has-session", "-t", SESSION], capture_output=True, text=True, timeout=5)
    if present.returncode:
        # The launcher rejects other live inputs and concurrent session creation.
        run(["bash", str(root / "bash/start_tianji_pico_teleop.sh"), "--detach",
             "--calibration-dir", str(revision), "--pico-world-x-offset", "0.20"], check=True)
    def option(name):
        return run(["tmux", "show-options", "-t", SESSION, "-v", name],
                   capture_output=True, text=True, check=True, timeout=5).stdout.strip()
    if (option("@tianji_checkout") != str(root.resolve()) or
        option("@tianji_calibration_dir") != str(revision) or
        option("@tianji_calibration_sha256") != expected or fingerprint(revision) != expected):
        raise RuntimeError("已有 PICO 会话的工程、人员、标定版本或内容不匹配（或缺少身份标记）；请显式停止后重试。")
    run([sys.executable, str(Path(__file__).with_name("check_pico_session_ready.py")),
         "--session", SESSION, "--checkout", str(root.resolve()), "--timeout-s", "30"], check=True)
    print(f"PICO input verified: user={user}, revision={revision.name}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--user")
    group.add_argument("--fingerprint", type=Path)
    group.add_argument("--release-owner")
    args = parser.parse_args()
    try:
        if args.release_owner:
            release_owner(args.release_owner)
        elif args.fingerprint:
            print(fingerprint(args.fingerprint))
        else:
            ensure(args.user)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(2, str(error) + "\n")


if __name__ == "__main__":
    main()
