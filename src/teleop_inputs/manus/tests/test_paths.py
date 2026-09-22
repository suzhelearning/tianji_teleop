"""Personnel calibration pairs must not alias another user's data."""

from pathlib import Path

import pytest

from manus_bridge.list_calibrations import calibration_users
from manus_bridge.paths import calibration_dir


@pytest.fixture()
def personnel(tmp_path, monkeypatch):
    root = tmp_path / "workspace"
    root.mkdir()
    (root / "pixi.toml").touch()
    monkeypatch.setenv("TIANJI_WORKSPACE", str(root))
    return root


def pair(directory: Path, user: str):
    directory.mkdir(parents=True, exist_ok=True)
    for side in ("Left", "Right"):
        (directory / f"{user}{side}MetaglovePro.mcal").write_text("fixture")


def test_lists_only_complete_personnel_pairs_without_profile_yaml(personnel):
    for user in ("zoe", "alice"):
        directory = personnel / "profiles" / user / "manus"
        pair(directory, user)
        assert calibration_dir(user) == directory
    partial = personnel / "profiles/partial/manus"
    partial.mkdir(parents=True)
    (partial / "partialLeftMetaglovePro.mcal").write_text("fixture")
    (partial / "partialRightMetaglovePro.mcal").mkdir()
    pair(personnel / "profiles/wrong-name/manus", "someone-else")
    pair(personnel / "profiles/invalid.name/manus", "invalid.name")
    assert calibration_users() == ["alice", "zoe"]
    with pytest.raises(ValueError):
        calibration_dir("partial")


@pytest.mark.parametrize("user", ["../alice", "/alice", "", "a/b", "a.b"])
def test_rejects_unsafe_user_names(personnel, user):
    with pytest.raises(ValueError):
        calibration_dir(user)


@pytest.mark.parametrize("level", ["profiles", "person", "manus", "file"])
def test_rejects_symlink_escape_at_each_boundary(personnel, level):
    profiles = personnel / "profiles"
    selected = profiles / "alice"
    directory = selected / "manus"
    other = personnel / "other-person"
    pair(other / "manus", "alice")
    if level == "profiles":
        external = personnel.parent / "external-profiles"
        pair(external / "alice/manus", "alice")
        profiles.symlink_to(external, target_is_directory=True)
    elif level == "person":
        profiles.mkdir()
        selected.symlink_to(other, target_is_directory=True)
    elif level == "manus":
        selected.mkdir(parents=True)
        directory.symlink_to(other / "manus", target_is_directory=True)
    else:
        pair(directory, "alice")
        left = directory / "aliceLeftMetaglovePro.mcal"
        left.unlink()
        left.symlink_to(other / "manus/aliceLeftMetaglovePro.mcal")
    with pytest.raises(ValueError):
        calibration_dir("alice")
    assert calibration_users() == []


def test_missing_pair_never_uses_legacy_calibration(personnel):
    pair(personnel / "src/teleop_inputs/manus/calibration", "kj")
    pair(personnel / "install/default/share/manus_bridge/calibration", "kj")
    with pytest.raises(ValueError):
        calibration_dir("kj")
    assert calibration_users() == []
