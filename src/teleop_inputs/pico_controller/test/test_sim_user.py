"""Offline user/session selection; no SDK, ROS or live tmux operations."""
from pathlib import Path
import os
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from ensure_pico_user import ensure, fingerprint, release_owner


def test_cleanup_refuses_other_checkout(tmp_path):
    _, commands, run = fixture(tmp_path, mismatch='@tianji_checkout')
    with pytest.raises(RuntimeError, match='checkout changed'):
        release_owner('a'*32, root=tmp_path, run=run)
    assert not any('if-shell' in c for c in commands)


def test_cleanup_rejects_invalid_token(tmp_path):
    with pytest.raises(ValueError):
        release_owner('bad; kill-server', root=tmp_path)


def test_owner_cleanup_on_isolated_tmux_server(tmp_path):
    import shutil
    import uuid
    if not shutil.which('tmux'):
        pytest.skip('tmux is required for isolated server test')
    socket_name = 'pico-owner-test-' + uuid.uuid4().hex
    def run(command, **kwargs):
        return subprocess.run(['tmux', '-L', socket_name, *command[1:]], **kwargs)
    try:
        run(['tmux', 'new-session', '-d', '-s', 'pico_tianji_teleop', 'sleep 60'], check=True)
        run(['tmux', 'set-option', '-t', 'pico_tianji_teleop', '@tianji_checkout', str(tmp_path)], check=True)
        # Existing unowned and replacement-owned sessions both survive.
        release_owner('a'*32, root=tmp_path, run=run)
        assert run(['tmux', 'has-session', '-t', 'pico_tianji_teleop']).returncode == 0
        run(['tmux', 'set-option', '-t', 'pico_tianji_teleop', '@tianji_sim_owner', 'b'*32], check=True)
        release_owner('a'*32, root=tmp_path, run=run)
        assert run(['tmux', 'has-session', '-t', 'pico_tianji_teleop']).returncode == 0
        release_owner('b'*32, root=tmp_path, run=run)
        assert run(['tmux', 'has-session', '-t', 'pico_tianji_teleop'], capture_output=True).returncode != 0
    finally:
        run(['tmux', 'kill-server'], capture_output=True)


def fixture(tmp_path, present=True, mismatch=None, ready_error=False):
    revision = tmp_path / "profiles/person/pico-simple/cal-test"
    revision.mkdir(parents=True)
    for side in ("left", "right"):
        for kind in ("palm_tcp", "wrist_pivot", "arm_geometry"):
            (revision / f"pico_{side}_{kind}.yaml").write_text("synthetic hash fixture")
    options = {"@tianji_checkout": str(tmp_path),
               "@tianji_calibration_dir": str(revision),
               "@tianji_calibration_sha256": fingerprint(revision)}
    if mismatch:
        options[mismatch] = "unknown/other"
    commands = []
    def run(command, **kwargs):
        commands.append(command)
        if command[:2] == ["tmux", "has-session"]:
            return subprocess.CompletedProcess(command, 0 if present else 1, "")
        if command[:2] == ["tmux", "show-options"]:
            return subprocess.CompletedProcess(command, 0, options.get(command[-1], ""))
        if "check_pico_session_ready.py" in command[1] and ready_error:
            raise subprocess.CalledProcessError(2, command)
        return subprocess.CompletedProcess(command, 0, "")
    return revision, commands, run


@pytest.mark.parametrize("present", [True, False])
def test_matching_or_new_input_requires_readiness(tmp_path, present):
    revision, commands, run = fixture(tmp_path, present)
    ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision, discover=lambda: None)
    starts = [c for c in commands if c[0] == "bash"]
    assert len(starts) == (0 if present else 1)
    assert "check_pico_session_ready.py" in commands[-1][1]
    assert not any("kill-session" in c for c in commands)


@pytest.mark.parametrize("key", ["@tianji_checkout", "@tianji_calibration_dir", "@tianji_calibration_sha256"])
def test_mismatch_does_not_replace_existing_session(tmp_path, key):
    revision, commands, run = fixture(tmp_path, mismatch=key)
    with pytest.raises(RuntimeError, match="不匹配"):
        ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision, discover=lambda: None)
    assert not any(c[0] == "bash" or "kill-session" in c for c in commands)


def test_stale_session_cannot_be_reused(tmp_path):
    revision, commands, run = fixture(tmp_path, ready_error=True)
    with pytest.raises(subprocess.CalledProcessError):
        ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision, discover=lambda: None)


def test_missing_person_profile_never_starts_input(tmp_path):
    def fail(*args, **kwargs):
        pytest.fail("must resolve profile before invoking commands")
    with pytest.raises(ValueError, match="missing or inaccessible"):
        ensure("missing", root=tmp_path, run=fail)


def test_fingerprint_detects_bundle_changes(tmp_path):
    revision, _, _ = fixture(tmp_path)
    before = fingerprint(revision)
    (revision / "pico_right_palm_tcp.yaml").write_text("changed")
    assert fingerprint(revision) != before


def test_simulation_reuses_matching_foreground_without_starting_or_stopping_it(tmp_path):
    revision, commands, run = fixture(tmp_path)
    active = {"checkout": str(tmp_path), "calibration_dir": str(revision),
              "calibration_sha256": fingerprint(revision), "owner": "a" * 32, "pid": 123}
    ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision,
           discover=lambda: dict(active))
    assert len(commands) == 1
    assert "--viewer-owner" in commands[0]
    assert not any("tmux" in command or command[0] == "bash" for command in commands)


def test_foreground_reuse_refuses_other_calibration_and_restart_during_readiness(tmp_path):
    revision, commands, run = fixture(tmp_path)
    active = {"checkout": str(tmp_path), "calibration_dir": str(revision),
              "calibration_sha256": fingerprint(revision), "owner": "a" * 32, "pid": 123}
    with pytest.raises(RuntimeError):
        ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision,
               discover=lambda: dict(active, calibration_sha256="changed"))
    assert not commands
    observations = iter([active, dict(active, owner="b" * 32)])
    with pytest.raises(RuntimeError):
        ensure("person", root=tmp_path, run=run, resolver=lambda *_: revision,
               discover=lambda: next(observations))
    assert not any("kill-session" in command for command in commands)

