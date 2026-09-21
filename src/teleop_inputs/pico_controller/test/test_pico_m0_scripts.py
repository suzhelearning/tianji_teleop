"""Process-group cleanup and M0 domain locking.

These exercise `scripts/pico_process_cleanup.sh`, which `start_pico_m0_runtime.sh`
sources and installs alongside the node.
"""

from pathlib import Path
import shlex
import subprocess
import sys


PACKAGE_ROOT = Path(__file__).parents[1]
PROCESS_CLEANUP = PACKAGE_ROOT / "scripts" / "pico_process_cleanup.sh"


def test_process_group_cleanup_returns_even_when_child_ignores_soft_signals(tmp_path):
    ready = tmp_path / "ready"
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      setsid bash -c 'trap "" INT TERM; touch "$1"; while :; do sleep 30; done' bash {shlex.quote(str(ready))} &
      leader_pid=$!
      trap 'kill -KILL -- "-$leader_pid" 2>/dev/null || true; wait "$leader_pid" 2>/dev/null || true' EXIT
      for _ in {{1..200}}; do
        [[ -f {shlex.quote(str(ready))} ]] && break
        sleep 0.01
      done
      [[ -f {shlex.quote(str(ready))} ]]
      pico_stop_process_group "$leader_pid"
      echo cleanup-complete
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=10.0,
        check=False,
    )

    assert result.returncode == 0
    assert "cleanup-complete" in result.stdout


def test_process_group_cleanup_kills_descendant_after_leader_exits(tmp_path):
    child_file = tmp_path / "descendant.pid"
    worker = tmp_path / "process_group_worker.sh"
    worker.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
child_file="$1"
trap 'exit 0' INT
bash -c 'trap "" INT TERM; while :; do sleep 30; done' >/dev/null 2>&1 &
echo "$!" > "$child_file"
while :; do sleep 30; done
""",
        encoding="utf-8",
    )
    worker.chmod(0o755)
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      child_file={shlex.quote(str(child_file))}
      setsid {shlex.quote(str(worker))} "$child_file" &
      leader_pid=$!
      for _ in {{1..100}}; do
        [[ -s "$child_file" ]] && break
        sleep 0.01
      done
      descendant_pid="$(cat "$child_file")"
      pico_stop_process_group "$leader_pid"
      if /usr/bin/ps -o stat= -p "$descendant_pid" 2>/dev/null | grep -qv '^[[:space:]]*Z'; then
        echo "descendant-still-running:$descendant_pid" >&2
        exit 9
      fi
      echo cleanup-complete
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=5.0,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "cleanup-complete" in result.stdout


def test_process_group_cleanup_honors_extended_grace_period(tmp_path):
    marker = tmp_path / "graceful.marker"
    worker = tmp_path / "graceful_worker.py"
    ready = tmp_path / "ready"
    worker.write_text(
        f"""import signal
import time
from pathlib import Path

marker = Path({str(marker)!r})

def handle_interrupt(_signum, _frame):
    time.sleep(1.2)
    marker.touch()
    raise SystemExit(0)

signal.signal(signal.SIGINT, handle_interrupt)
Path({str(ready)!r}).touch()
while True:
    time.sleep(30)
""",
        encoding="utf-8",
    )
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      setsid {shlex.quote(sys.executable)} {shlex.quote(str(worker))} &
      leader_pid=$!
      trap 'kill -KILL -- "-$leader_pid" 2>/dev/null || true; wait "$leader_pid" 2>/dev/null || true' EXIT
      for _ in {{1..200}}; do
        [[ -f {shlex.quote(str(ready))} ]] && break
        sleep 0.01
      done
      [[ -f {shlex.quote(str(ready))} ]]
      pico_stop_process_group "$leader_pid" 40 10
      [[ -f {shlex.quote(str(marker))} ]]
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=10.0,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_equivalent_domain_spellings_contend_for_the_same_lock(tmp_path):
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      first="$(pico_normalize_ros_domain 120)"
      alias="$(pico_normalize_ros_domain 0120)"
      [[ "$first" == "$alias" ]]
      exec {{first_fd}}>"{tmp_path}/pico_m0_domain_${{first}}.lock"
      exec {{alias_fd}}>"{tmp_path}/pico_m0_domain_${{alias}}.lock"
      flock -n "$first_fd"
      if flock -n "$alias_fd"; then
        echo "equivalent domain alias acquired a second lock" >&2
        exit 9
      fi
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
