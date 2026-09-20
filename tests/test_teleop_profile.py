from pathlib import Path
import fcntl
import os
import shutil
import signal
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from teleop_profile import available_users, calibrate_profile, resolve_profile


@pytest.fixture
def profile_root(tmp_path):
    from pico_test_data import synthetic_profile
    synthetic_profile(tmp_path)
    return tmp_path


def _change_profile(root, **changes):
    path = root / "profiles" / "syz" / "profile.yaml"
    document = yaml.safe_load(path.read_text())
    document.update(changes)
    path.write_text(yaml.safe_dump(document))


def test_profile_identity_must_match_selected_person(profile_root):
    _change_profile(profile_root, user_id="someone_else")
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)
    assert available_users(profile_root) == []


def test_person_selection_cannot_escape_profiles(profile_root):
    with pytest.raises(ValueError):
        resolve_profile("../syz", "pico", root=profile_root)


def test_profile_directory_cannot_alias_another_person(profile_root):
    (profile_root / "profiles" / "other").symlink_to("syz", target_is_directory=True)
    with pytest.raises(ValueError):
        resolve_profile("other", "pico", root=profile_root)
    assert available_users(profile_root) == ["syz"]


def test_active_set_is_one_path_component(profile_root):
    _change_profile(profile_root, pico={"active_set": "../20260908-01"})
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)


def test_snapshot_symlink_cannot_redirect_selected_revision(profile_root):
    pico = profile_root / "profiles" / "syz" / "pico"
    (pico / "20260908-01").rename(pico / "older")
    (pico / "20260908-01").symlink_to("older", target_is_directory=True)
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)


def test_artifact_symlink_cannot_escape_snapshot(profile_root):
    snapshot = profile_root / "profiles" / "syz" / "pico" / "20260908-01"
    wrist = snapshot / "pico_left_wrist_pivot.yaml"
    external = profile_root / wrist.name
    wrist.rename(external)
    wrist.symlink_to(external)
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)


@pytest.mark.parametrize("side", ["left", "right"])
def test_changed_wrist_bytes_break_geometry_chain(profile_root, side):
    snapshot = profile_root / "profiles" / "syz" / "pico" / "20260908-01"
    wrist = snapshot / f"pico_{side}_wrist_pivot.yaml"
    with wrist.open("a") as stream:
        stream.write("\n# Different calibration bytes\n")
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)


def test_missing_snapshot_never_uses_another_revision(profile_root):
    pico = profile_root / "profiles" / "syz" / "pico"
    (pico / "20260908-01").rename(pico / "older")
    with pytest.raises(ValueError):
        resolve_profile("syz", "pico", root=profile_root)


def test_pico_does_not_require_manus_component_or_files(profile_root):
    _change_profile(profile_root, manus=None)
    expected = profile_root / "profiles" / "syz" / "pico" / "20260908-01"
    assert resolve_profile("syz", "pico", root=profile_root) == str(expected)


def test_manus_does_not_require_pico_component_or_files(profile_root):
    _change_profile(profile_root, pico=None)
    shutil.rmtree(profile_root / "profiles" / "syz" / "pico")
    calibration = profile_root / "manus" / "calibration"
    calibration.mkdir(parents=True)
    for side in ("Left", "Right"):
        name = f"syz{side}MetaglovePro.mcal"
        (calibration / name).touch()
    assert resolve_profile("syz", "manus", root=profile_root) == "syz"
    (calibration / "syzRightMetaglovePro.mcal").unlink()
    with pytest.raises(ValueError):
        resolve_profile("syz", "manus", root=profile_root)


def test_manus_rejects_abbreviated_user_override_before_starting(tmp_path):
    launcher = tmp_path / "manus.sh"
    shutil.copyfile(ROOT / "manus.sh", launcher)
    interpreter = tmp_path / ".venv/bin/python"
    interpreter.parent.mkdir(parents=True)
    (tmp_path / "scripts").mkdir()
    shutil.copyfile(ROOT / "scripts/environment.sh", tmp_path / "scripts/environment.sh")
    # Reaching the profile resolver would mean the override crossed the guard.
    # This sentinel also prevents any actual capture/hardware process in regressions.
    interpreter.write_text("#!/usr/bin/env bash\nexit 73\n")
    interpreter.chmod(0o755)
    result = subprocess.run(
        ["bash", str(launcher), "--user", "syz", "--us", "another"],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2


def _copy_side_command(source, side, *, exit_code=0):
    return [
        sys.executable, "-c",
        "import os,pathlib,shutil,sys; "
        "source=pathlib.Path(sys.argv[1]); "
        "draft=pathlib.Path(os.environ['PICO_CALIBRATION_DIR']); "
        "[shutil.copyfile(path,draft/path.name) "
        "for path in source.glob('pico_'+sys.argv[2]+'_*.yaml')]; "
        "recordings=pathlib.Path(os.environ['PICO_CALIBRATION_RECORDINGS_DIR']); "
        "(recordings/(sys.argv[2]+'.jsonl')).write_text('recorded progress'); "
        "sys.exit(int(sys.argv[3]))",
        str(source), side, str(exit_code),
    ]


def test_new_person_publishes_only_after_both_sides(profile_root, monkeypatch):
    source = Path(resolve_profile("syz", "pico", root=profile_root))
    outer = profile_root / "outer-selection"
    outer.mkdir()
    monkeypatch.setenv("PICO_CALIBRATION_DIR", str(outer))
    person = profile_root / "profiles" / "zjx"
    assert calibrate_profile(
        "zjx", _copy_side_command(source, "left"), root=profile_root,
    ) == 0
    draft = person / "pico" / ".draft"
    left = (draft / "pico_left_arm_geometry.yaml").read_bytes()
    assert "zjx" not in available_users(profile_root)
    assert not (person / "profile.yaml").exists()
    assert not (draft / "pico_right_arm_geometry.yaml").exists()
    assert list(outer.iterdir()) == []
    assert calibrate_profile(
        "zjx", _copy_side_command(source, "right"), root=profile_root,
    ) == 0
    revision = Path(resolve_profile("zjx", "pico", root=profile_root))
    assert (revision / "pico_left_arm_geometry.yaml").read_bytes() == left
    assert not draft.exists()
    assert (person / "recordings" / "left.jsonl").read_text() == "recorded progress"
    assert (person / "recordings" / "right.jsonl").read_text() == "recorded progress"
    profile = yaml.safe_load((person / "profile.yaml").read_text())
    assert profile["manus"] == {"user": "zjx"}
    assert not (profile_root / "manus").exists()


def test_failed_calibration_preserves_draft_and_exit_code(profile_root):
    source = Path(resolve_profile("syz", "pico", root=profile_root))
    assert calibrate_profile(
        "zjx", _copy_side_command(source, "left", exit_code=23), root=profile_root,
    ) == 23
    person = profile_root / "profiles" / "zjx"
    assert (person / "pico" / ".draft" / "pico_left_arm_geometry.yaml").is_file()
    assert not (person / "profile.yaml").exists()
    assert calibrate_profile(
        "zjx", _copy_side_command(source, "right"), root=profile_root,
    ) == 0
    assert Path(resolve_profile("zjx", "pico", root=profile_root)).is_dir()


def test_invalid_draft_keeps_old_revision_and_mapped_manus(profile_root):
    _change_profile(profile_root, manus={"user": "mapped"})
    person = profile_root / "profiles" / "syz"
    profile_bytes = (person / "profile.yaml").read_bytes()
    old = Path(resolve_profile("syz", "pico", root=profile_root))
    old_bytes = {path.name: path.read_bytes() for path in old.glob("*.yaml")}
    corrupt = [
        sys.executable, "-c",
        "import os,pathlib; "
        "p=pathlib.Path(os.environ['PICO_CALIBRATION_DIR'])/'pico_left_wrist_pivot.yaml'; "
        "p.write_bytes(p.read_bytes()+b'\\n# stale geometry hash\\n')",
    ]
    assert calibrate_profile("syz", corrupt, root=profile_root) == 0
    assert (person / "profile.yaml").read_bytes() == profile_bytes
    assert resolve_profile("syz", "pico", root=profile_root) == str(old)
    assert {path.name: path.read_bytes() for path in old.glob("*.yaml")} == old_bytes
    # Resume the existing draft rather than replacing it from the active snapshot.
    assert calibrate_profile(
        "syz", [sys.executable, "-c", "pass"], root=profile_root,
    ) == 0
    assert (person / "profile.yaml").read_bytes() == profile_bytes
    assert calibrate_profile(
        "syz", _copy_side_command(old, "left"), root=profile_root,
    ) == 0
    new = Path(resolve_profile("syz", "pico", root=profile_root))
    assert new != old
    assert yaml.safe_load((person / "profile.yaml").read_text())["manus"] == {"user": "mapped"}
    assert {path.name: path.read_bytes() for path in old.glob("*.yaml")} == old_bytes
    assert not (person / "pico" / ".draft").exists()


def test_same_person_lock_rejects_concurrent_calibration(profile_root):
    person = profile_root / "profiles" / "syz"
    marker = profile_root / "child-started"
    with (person / ".calibration.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ValueError, match="already running"):
            calibrate_profile(
                "syz",
                [sys.executable, "-c", "import pathlib,sys; pathlib.Path(sys.argv[1]).touch()", str(marker)],
                root=profile_root,
            )
    assert not marker.exists()
    assert not (person / "pico" / ".draft").exists()


def test_calibration_rejects_name_before_creating_paths(tmp_path):
    with pytest.raises(ValueError):
        calibrate_profile("../escape", [sys.executable, "-c", "pass"], root=tmp_path)
    assert list(tmp_path.iterdir()) == []


def test_invalid_active_revision_is_not_seeded(profile_root):
    person = profile_root / "profiles" / "syz"
    old = Path(resolve_profile("syz", "pico", root=profile_root))
    (old / "pico_left_palm_tcp.yaml").unlink()
    with pytest.raises(ValueError):
        calibrate_profile("syz", [sys.executable, "-c", "pass"], root=profile_root)
    assert not (person / "pico" / ".draft").exists()


@pytest.mark.parametrize("entry", ["pico", "pico/.draft", "recordings", ".calibration.lock"])
def test_calibration_rejects_symlinked_writable_paths(profile_root, entry):
    person = profile_root / "profiles" / "zjx"
    (person / "pico").mkdir(parents=True)
    target = profile_root / "outside"
    target.mkdir()
    path = person / entry
    if path.is_dir():
        path.rmdir()
    path.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError):
        calibrate_profile("zjx", [sys.executable, "-c", "pass"], root=profile_root)
    assert list(target.iterdir()) == []


def test_draft_recording_symlink_is_not_published(profile_root):
    source = Path(resolve_profile("syz", "pico", root=profile_root))
    assert calibrate_profile("zjx", _copy_side_command(source, "left"), root=profile_root) == 0
    person = profile_root / "profiles" / "zjx"
    outside = profile_root / "outside"
    outside.write_text("untouched")
    (person / "recordings" / "escape").symlink_to(outside)
    with pytest.raises(ValueError):
        calibrate_profile("zjx", _copy_side_command(source, "right"), root=profile_root)
    assert not (person / "profile.yaml").exists()
    assert outside.read_text() == "untouched"


def test_profile_replace_failure_preserves_previous_selection(profile_root, monkeypatch):
    person = profile_root / "profiles" / "syz"
    previous = (person / "profile.yaml").read_bytes()
    original_replace = os.replace

    def fail_profile_replace(source, destination):
        if Path(destination) == person / "profile.yaml":
            raise OSError("simulated interrupted profile replacement")
        return original_replace(source, destination)

    monkeypatch.setattr(os, "replace", fail_profile_replace)
    with pytest.raises(OSError, match="interrupted"):
        calibrate_profile("syz", [sys.executable, "-c", "pass"], root=profile_root)
    assert (person / "profile.yaml").read_bytes() == previous
    assert (person / "pico" / ".draft" / "pico_right_arm_geometry.yaml").is_file()


@pytest.mark.parametrize("arguments", [
    ["--user", "zjx", "--calibrate"],
    ["--calibrate", "--", "true"],
    ["--user", "zjx", "--calibrate", "--component", "pico", "--", "true"],
    ["--list-users", "--calibrate", "--user", "zjx", "--", "true"],
    ["--user", "zjx", "--component", "pico", "--", "true"],
    ["--list-users", "--component", "pico"],
])
def test_calibration_cli_rejects_incompatible_modes(arguments):
    result = subprocess.run(
        [sys.executable, str(ROOT / "teleop_profile.py"), *arguments],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_interrupt_is_forwarded_and_keeps_draft(profile_root, signum):
    command = [
        sys.executable, "-c",
        "import os,pathlib,signal,sys; "
        "draft=pathlib.Path(os.environ['PICO_CALIBRATION_DIR']); "
        "signal.signal(int(sys.argv[1]), lambda number,frame: "
        "((draft/'interrupted').write_text(str(number)),sys.exit(0))); "
        "os.kill(os.getppid(),int(sys.argv[1])); signal.pause()",
        str(signum),
    ]
    assert calibrate_profile("zjx", command, root=profile_root) == 128 + signum
    person = profile_root / "profiles" / "zjx"
    assert (person / "pico" / ".draft" / "interrupted").read_text() == str(signum)
    assert not (person / "profile.yaml").exists()
    assert calibrate_profile("zjx", [sys.executable, "-c", "pass"], root=profile_root) == 0


def test_process_death_during_publication_recovers_progress(profile_root):
    person = profile_root / "profiles" / "syz"
    previous = (person / "profile.yaml").read_bytes()
    child = (
        "import os,pathlib; "
        "p=pathlib.Path(os.environ['PICO_CALIBRATION_DIR'])/'progress'; "
        "p.write_text('survived interruption')"
    )
    script = """
import os
from pathlib import Path
import sys
from teleop_profile import calibrate_profile
root = Path(sys.argv[1])
replace = os.replace
def interrupt_profile(source, destination):
    if Path(destination) == root / 'profiles/syz/profile.yaml':
        os._exit(77)
    return replace(source, destination)
os.replace = interrupt_profile
calibrate_profile('syz', [sys.executable, '-c', sys.argv[2]], root=root)
"""
    result = subprocess.run(
        [sys.executable, "-c", script, str(profile_root), child],
        cwd=ROOT, capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 77
    assert (person / "profile.yaml").read_bytes() == previous
    assert calibrate_profile("syz", [sys.executable, "-c", "pass"], root=profile_root) == 0
    revision = Path(resolve_profile("syz", "pico", root=profile_root))
    assert (revision / "progress").read_text() == "survived interruption"
    assert not (person / "pico" / ".draft").exists()


def test_manus_only_person_can_publish_first_pico_revision(profile_root):
    source = Path(resolve_profile("syz", "pico", root=profile_root))
    person = profile_root / "profiles" / "zjx"
    person.mkdir()
    profile = {"schema_version": 1, "user_id": "zjx", "manus": {"user": "mapped"}}
    (person / "profile.yaml").write_text(yaml.safe_dump(profile))
    original = (person / "profile.yaml").read_bytes()
    assert calibrate_profile("zjx", _copy_side_command(source, "left"), root=profile_root) == 0
    assert (person / "profile.yaml").read_bytes() == original
    assert not (person / "pico" / ".draft" / "pico_right_arm_geometry.yaml").exists()
    assert calibrate_profile("zjx", _copy_side_command(source, "right"), root=profile_root) == 0
    revision = Path(resolve_profile("zjx", "pico", root=profile_root))
    assert (revision / "pico_right_arm_geometry.yaml").read_bytes() == (source / "pico_right_arm_geometry.yaml").read_bytes()
    assert yaml.safe_load((person / "profile.yaml").read_text())["manus"] == {"user": "mapped"}


@pytest.mark.parametrize("selection", [None, {}, {"active_set": "../other"}])
def test_present_invalid_pico_selection_is_not_treated_as_manus_only(profile_root, selection):
    _change_profile(profile_root, pico=selection)
    with pytest.raises(ValueError):
        calibrate_profile("syz", [sys.executable, "-c", "pass"], root=profile_root)
    assert not (profile_root / "profiles" / "syz" / "pico" / ".draft").exists()


def test_ignored_interrupt_is_killed_after_grace_and_reaped(profile_root):
    person = profile_root / "profiles" / "zjx"
    pid_file = person / "pico" / ".draft" / "child-pid"
    child = (
        "import os,pathlib,signal,time; "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
        "p=pathlib.Path(os.environ['PICO_CALIBRATION_DIR'])/'child-pid'; "
        "p.write_text(str(os.getpid())); "
        "os.kill(os.getppid(),signal.SIGTERM); time.sleep(60)"
    )
    script = """
from pathlib import Path
import sys
import teleop_profile
teleop_profile._INTERRUPT_GRACE_SECONDS = 0.1
raise SystemExit(teleop_profile.calibrate_profile(
    'zjx', [sys.executable, '-c', sys.argv[2]], root=Path(sys.argv[1]),
))
"""
    try:
        result = subprocess.run(
            [sys.executable, "-c", script, str(profile_root), child],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5,
        )
        assert result.returncode == 128 + signal.SIGTERM
        assert not (person / "profile.yaml").exists()
        with pytest.raises(ProcessLookupError):
            os.kill(int(pid_file.read_text()), 0)
        assert calibrate_profile("zjx", [sys.executable, "-c", "pass"], root=profile_root) == 0
    finally:
        if pid_file.exists():
            try:
                os.killpg(int(pid_file.read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_leader_exit_does_not_kill_detached_publisher_cleanup(profile_root, monkeypatch):
    import teleop_profile

    monkeypatch.setattr(teleop_profile, "_INTERRUPT_GRACE_SECONDS", 2.0)
    draft = profile_root / "profiles" / "zjx" / "pico" / ".draft"
    cleanup_child = """
import os, pathlib, signal, subprocess, sys, time
draft = pathlib.Path(os.environ['PICO_CALIBRATION_DIR'])
publisher = subprocess.Popen(
    [sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True,
)
(draft / 'publisher-pid').write_text(str(publisher.pid))
def cleanup(signum, frame):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    time.sleep(0.1)
    publisher.terminate()
    publisher.wait(timeout=5)
    (draft / 'publisher-cleaned').touch()
    raise SystemExit(0)
signal.signal(signal.SIGTERM, cleanup)
os.kill(int(sys.argv[1]), signal.SIGTERM)
signal.pause()
"""
    leader = "import subprocess,sys; subprocess.run([sys.executable,'-c',sys.argv[1],sys.argv[2]])"
    try:
        result = calibrate_profile(
            "zjx", [sys.executable, "-c", leader, cleanup_child, str(os.getpid())], root=profile_root,
        )
        assert result == 128 + signal.SIGTERM
        assert (draft / "publisher-cleaned").exists()
        with pytest.raises(ProcessLookupError):
            os.kill(int((draft / "publisher-pid").read_text()), 0)
    finally:
        if (draft / "publisher-pid").exists():
            try:
                os.killpg(int((draft / "publisher-pid").read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass
