from pathlib import Path
import os
import shlex
import subprocess
import shutil
import sys


PACKAGE_ROOT = Path(__file__).parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
PROCESS_CLEANUP = PACKAGE_ROOT / "scripts" / "pico_process_cleanup.sh"
TREMOR_RECORDER = REPO_ROOT / "scripts" / "record_pico_tremor.sh"
TIANJI_TMUX_LAUNCHER = REPO_ROOT / "scripts" / "start_tianji_pico_teleop.sh"
TIANJI_TMUX_STOPPER = REPO_ROOT / "scripts" / "stop_tianji_pico_teleop.sh"


def _write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_tianji_launcher_environment(tmp_path: Path) -> dict[str, str]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    tmux_state = tmp_path / "tmux-session"
    tmux_log = tmp_path / "tmux.log"
    project = tmp_path / "project"
    tracking = project / "tracking"
    scripts = tracking / "scripts"
    scripts.mkdir(parents=True)
    for source in (TIANJI_TMUX_LAUNCHER, TIANJI_TMUX_STOPPER, REPO_ROOT / "scripts/environment.sh"):
        shutil.copy2(source, scripts / source.name)
    overlay = tracking / "install"
    overlay.mkdir()
    (overlay / "local_setup.bash").write_text("")
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("")
    venv_bin = project / ".venv/bin"
    venv_bin.mkdir(parents=True)
    _write_executable(
        venv_bin / "python",
        '#!/usr/bin/env bash\n[[ "$1" == "-c" ]] && exit 0\nexit 2\n',
    )

    _write_executable(
        fake_bin / "tmux",
        """#!/usr/bin/env bash
set -euo pipefail
printf '%q ' "$@" >> "$FAKE_TMUX_LOG"
printf '\n' >> "$FAKE_TMUX_LOG"
case "${1:-}" in
  has-session)
    [[ -f "$FAKE_TMUX_STATE" ]]
    ;;
  new-session)
    touch "$FAKE_TMUX_STATE"
    ;;
  list-windows)
    [[ -f "$FAKE_TMUX_STATE" ]]
    printf '0: driver\n1: m0\n2: bridge\n'
    ;;
  kill-session)
    rm -f "$FAKE_TMUX_STATE"
    ;;
  *)
    :
    ;;
esac
""",
    )
    _write_executable(
        fake_bin / "adb",
        """#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "get-state" ]] || exit 2
printf '%s\n' "${FAKE_ADB_STATE:-device}"
""",
    )
    _write_executable(
        venv_bin / "python3",
        """#!/usr/bin/env bash
set -euo pipefail
printf 'cleanup %q\n' "$@" >> "$FAKE_TMUX_LOG"
exit "${FAKE_CLEANUP_EXIT:-0}"
""",
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ROS_SETUP": str(ros_setup),
            "TIANJI_PYTHON": str(venv_bin / "python"),
            "FAKE_LAUNCHER": str(scripts / TIANJI_TMUX_LAUNCHER.name),
            "FAKE_STOPPER": str(scripts / TIANJI_TMUX_STOPPER.name),
            "FAKE_TMUX_STATE": str(tmux_state),
            "FAKE_TMUX_LOG": str(tmux_log),
        }
    )
    return environment




















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




































def test_tremor_recorder_rejects_non_positive_timing():
    for option in ("--duration", "--countdown"):
        result = subprocess.run(
            ["bash", str(TREMOR_RECORDER), option, "0", "--dry-run"],
            text=True,
            capture_output=True,
            check=False,
        )

        assert result.returncode == 2




















def test_tianji_tmux_stopper_stops_only_the_managed_session_idempotently(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    log_path = Path(environment["FAKE_TMUX_LOG"])

    started = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert started.returncode == 0, started.stderr

    for _ in range(2):
        stopped = subprocess.run(
            ["bash", environment["FAKE_STOPPER"]],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )
        assert stopped.returncode == 0, stopped.stderr

    final_log = log_path.read_text(encoding="utf-8")
    assert final_log.count("kill-session -t pico_tianji_teleop") == 1
    assert "kill-server" not in final_log


def test_tianji_tmux_launcher_restarts_one_fixed_session_without_duplicates(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    log_path = Path(environment["FAKE_TMUX_LOG"])

    first = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert first.returncode == 0, first.stderr

    first_log = log_path.read_text(encoding="utf-8")
    assert first_log.count("new-session") == 1
    assert "new-session -d -s pico_tianji_teleop -n driver" in first_log
    assert "new-window -t pico_tianji_teleop -n m0" in first_log
    assert "new-window -t pico_tianji_teleop -n bridge" in first_log

    second = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert second.returncode == 0, second.stderr
    second_log = log_path.read_text(encoding="utf-8")
    assert second_log.count("new-session") == 2
    assert second_log.count("kill-session -t pico_tianji_teleop") == 1
    assert second_log.count("cleanup ") == 2
    assert second_log.rindex("kill-session -t pico_tianji_teleop") < second_log.rindex(
        "cleanup "
    ) < second_log.rindex("new-session")

    status = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--status"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert status.returncode == 0, status.stderr
    assert "driver" in status.stdout
    assert "m0" in status.stdout
    assert "bridge" in status.stdout

    stopped = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--stop"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert stopped.returncode == 0, stopped.stderr
    final_log = log_path.read_text(encoding="utf-8")
    assert "kill-session -t pico_tianji_teleop" in final_log
    assert "kill-server" not in final_log
    assert final_log.count("cleanup ") == 2


def test_tianji_tmux_launcher_preserves_existing_session_when_adb_preflight_fails(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    state_path = Path(environment["FAKE_TMUX_STATE"])
    state_path.touch()
    environment["FAKE_ADB_STATE"] = "offline"
    log_path = Path(environment["FAKE_TMUX_LOG"])

    result = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )

    assert result.returncode == 2
    assert state_path.exists()
    log = log_path.read_text(encoding="utf-8") if log_path.exists() else ""
    assert "kill-session" not in log
    assert "cleanup " not in log


def test_tianji_tmux_launcher_fails_closed_when_historical_cleanup_fails(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    environment["FAKE_CLEANUP_EXIT"] = "2"
    log_path = Path(environment["FAKE_TMUX_LOG"])

    result = subprocess.run(
        ["bash", environment["FAKE_LAUNCHER"], "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )

    assert result.returncode == 2
    log = log_path.read_text(encoding="utf-8")
    assert "cleanup " in log
    assert "new-session" not in log
