#!/usr/bin/env python3
"""Select a published input; never switch an existing session's operator."""
import argparse
import os
import hashlib
from pathlib import Path
import subprocess
import shlex
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


def ensure(user, root=ROOT, run=subprocess.run, resolver=resolve, discover=None):
    revision = resolver(user, root)
    expected = fingerprint(revision)
    if discover is None:
        from pico_foreground import discover_foreground
        discover = discover_foreground
    active = discover()
    if active is not None:
        if (active["checkout"] != str(root.resolve())
                or active["calibration_dir"] != str(revision)
                or active["calibration_sha256"] != expected):
            raise RuntimeError("已有前台 PICO 的工程、人员或标定不匹配；请在其终端显式停止后重试。")
        run([sys.executable, str(Path(__file__).with_name("check_pico_session_ready.py")),
             "--viewer-owner", active["owner"], "--timeout-s", "30"], check=True)
        if fingerprint(revision) != expected or discover() != active:
            raise RuntimeError("前台 PICO 身份或标定在检查期间发生变化；拒绝复用。")
        print(f"PICO foreground input reused: user={user}, revision={revision.name}; "
              "its original terminal retains ownership", flush=True)
        return
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
    logs = run(["tmux", "show-options", "-q", "-t", SESSION, "-v", "@tianji_run_log_dir"],
               capture_output=True, text=True, timeout=5)
    if logs.returncode == 0 and logs.stdout.strip():
        print(f"PICO background logs: {logs.stdout.strip()}", flush=True)
    else:
        print("PICO session predates background logging; it was not restarted. "
              "Safely stop the executor before explicitly recreating the PICO session for node logs.",
              flush=True)
    run([sys.executable, str(Path(__file__).with_name("check_pico_session_ready.py")),
         "--session", SESSION, "--checkout", str(root.resolve()), "--timeout-s", "30"], check=True)
    print(f"PICO input verified: user={user}, revision={revision.name}", flush=True)


def foreground(user, root=ROOT):
    """Resolve one immutable personnel revision, then let this terminal own its input."""
    revision = resolve(user, root)
    fingerprint(revision)
    print(f"PICO foreground input: user={user}, revision={revision.name}", flush=True)
    command = ["bash", str(root / "bash/start_tianji_pico_teleop.sh"), "--foreground",
               "--calibration-dir", str(revision), "--pico-world-x-offset", "0.20"]
    # exec is essential: the console logger must signal the actual supervisor,
    # not a waiting intermediate process that could leave input children behind.
    os.execvp(command[0], command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--user")
    group.add_argument("--fingerprint", type=Path)
    group.add_argument("--release-owner")
    parser.add_argument("--foreground", action="store_true",
                        help="keep --user input and node diagnostics in this terminal")
    args = parser.parse_args()
    if args.foreground and not args.user:
        parser.error("--foreground requires --user")
    try:
        if args.release_owner:
            release_owner(args.release_owner)
        elif args.fingerprint:
            print(fingerprint(args.fingerprint))
        elif args.foreground:
            foreground(args.user)
        else:
            ensure(args.user)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        recovery = ""
        if args.user:
            recovery = (
                "\n恢复步骤（手动操作，不会自动停止进程）：\n"
                "  1. 若终端 4 正在运行，先在终端 4 按 Ctrl+C，确认机器人已停止、执行器已退出；"
                "不要在运动中重启 PICO。\n"
                f"  2. 前台输入在原终端按 Ctrl+C；后台输入在项目目录 {ROOT} 显式停止：\n"
                "     bash bash/run_stop_pico.sh\n"
                "  3. 重新启动 PICO：\n"
                f"     bash bash/run_pico.sh --user {shlex.quote(args.user)}\n"
            )
        parser.exit(2, str(error) + "\n" + recovery)


if __name__ == "__main__":
    main()
