import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.calibrate_pico_simple import build_bundle, calibrate, resolve
from pico_simple_artifact import LEFT_TCP, LEFT_WRIST, MANIFEST
from pico_calibration_artifact import validate_artifact
from pico_palm_tcp_runtime import load_tcp_transform, quaternion_to_matrix


@pytest.fixture
def bundle(tmp_path):
    from pico_test_data import synthetic_bundle
    source = synthetic_bundle(tmp_path / "synthetic")
    for name in (LEFT_TCP, LEFT_WRIST):
        shutil.copyfile(source / name, tmp_path / name)
    build_bundle(tmp_path, 1.7)
    return tmp_path


def test_runtime_bundle_loads_without_fake_quality(bundle):
    for side in ("left", "right"):
        tcp = bundle / f"pico_{side}_palm_tcp.yaml"
        wrist = bundle / f"pico_{side}_wrist_pivot.yaml"
        geometry = bundle / f"pico_{side}_arm_geometry.yaml"
        assert validate_artifact(tcp, "tcp", side).valid
        assert validate_artifact(wrist, "wrist", side).valid
        assert validate_artifact(geometry, "geometry", side, tcp_path=tcp, wrist_path=wrist).valid
        assert yaml.safe_load(geometry.read_text())["upper_arm_length_m"] == 1.7 * .155882
    left = load_tcp_transform(bundle / LEFT_TCP, "left")
    right = load_tcp_transform(bundle / "pico_right_palm_tcp.yaml", "right")
    m = np.diag([1, -1, 1])
    np.testing.assert_allclose(right.translation_m, m @ left.translation_m)
    np.testing.assert_allclose(quaternion_to_matrix(right.quaternion_xyzw),
        m @ quaternion_to_matrix(left.quaternion_xyzw) @ m, atol=1e-12)
    for name in ("pico_right_palm_tcp.yaml", "pico_right_wrist_pivot.yaml", "pico_left_arm_geometry.yaml"):
        assert "quality" not in yaml.safe_load((bundle / name).read_text())


@pytest.mark.parametrize("name", [LEFT_TCP, LEFT_WRIST, "pico_right_palm_tcp.yaml", "pico_right_wrist_pivot.yaml"])
def test_tampering_rejects_bundle(bundle, name):
    p = bundle / name
    doc = yaml.safe_load(p.read_text())
    doc["unexpected"] = "tampered"
    p.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError):
        load_tcp_transform(bundle / "pico_right_palm_tcp.yaml", "right")


def test_geometry_rejects_external_lineage(bundle):
    with pytest.raises(ValueError, match="exact bundle"):
        validate_artifact(bundle / "pico_right_arm_geometry.yaml", "geometry", "right",
                          tcp_path="elsewhere", wrist_path=bundle / "pico_right_wrist_pivot.yaml")


def fake_collector(command, env):
    assert command[-2] == "left"
    name = LEFT_TCP if command[-1] == "tcp" else LEFT_WRIST
    from pico_test_data import synthetic_bundle
    shutil.copyfile(synthetic_bundle(Path(env["PICO_CALIBRATION_DIR"]) / "synthetic") / name,
                    Path(env["PICO_CALIBRATION_DIR"]) / name)
    assert env["PICO_GEOMETRY_POLICY"] == "original"
    return 0


def test_new_user_publish_does_not_touch_legacy_and_failure_preserves_pointer(tmp_path):
    assert calibrate("new_user", 1.75, tmp_path, fake_collector, lambda _: "publish") == 0
    active = resolve("new_user", tmp_path)
    assert json.loads((active / MANIFEST).read_text())["height_m"] == 1.75
    person = tmp_path / "profiles/new_user"
    assert not (person / "profile.yaml").exists()
    # An existing legacy selection is neither read nor overwritten.
    (person / "profile.yaml").write_text("legacy untouched")
    pointer = (person / "pico-simple/active.json").read_bytes()
    assert calibrate("new_user", 1.8, tmp_path, fake_collector, lambda _: "cancel") == 0
    assert calibrate("new_user", 1.8, tmp_path, lambda *_: 7, lambda _: "publish") == 7
    assert pointer == (person / "pico-simple/active.json").read_bytes()
    assert (person / "profile.yaml").read_text() == "legacy untouched"


def test_new_lifecycle_collects_only_palm_tcp_and_derives_wrist(tmp_path):
    calls = []
    def collector(command, env):
        calls.append(command[-1])
        assert command[-1] == "tcp"
        return fake_collector(command, env)
    assert calibrate("new_user", 1.62, tmp_path, collector, lambda _: "publish") == 0
    assert calls == ["tcp"]
    revision = resolve("new_user", tmp_path)
    manifest = json.loads((revision / MANIFEST).read_text())
    assert manifest["schema_version"] == 2
    assert set(manifest["source_sha256"]) == {LEFT_TCP}
    for side in ("left", "right"):
        p = revision / f"pico_{side}_wrist_pivot.yaml"
        doc = yaml.safe_load(p.read_text())
        assert doc["wrist_to_palm_distance_m"] == pytest.approx(.05999994)
        assert doc["derivation"] == "height_template_not_measured"
        assert "quality" not in doc
        assert validate_artifact(p, "wrist", side).valid
    p.write_text(p.read_text().replace("0.05999994", "0.11933279"))
    with pytest.raises(ValueError):
        resolve("new_user", tmp_path)


@pytest.mark.parametrize("user,height", [("../escape", 1.7), ("ok", 170), ("ok", float("nan"))])
def test_bad_inputs_no_side_effects(tmp_path, user, height):
    with pytest.raises(ValueError):
        calibrate(user, height, tmp_path, lambda *_: pytest.fail("device called"))
    assert not (tmp_path / "profiles").exists()


def test_resolve_rejects_parent_alias(tmp_path):
    (tmp_path / "profiles").mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (tmp_path / "profiles/alias").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        resolve("alias", tmp_path)


def test_missing_pointer_never_selects_unpublished_revision(tmp_path):
    assert calibrate("new_user", 1.7, tmp_path, fake_collector, lambda _: "cancel") == 0
    base = tmp_path / "profiles/new_user/pico-simple"
    assert list(base.glob("cal-*"))
    with pytest.raises(FileNotFoundError, match="active.json"):
        resolve("new_user", tmp_path)
    assert not (base / "active.json").exists()


def test_lock_refuses_concurrent_legacy_calibration(tmp_path):
    import fcntl
    person = tmp_path / "profiles/user"
    person.mkdir(parents=True)
    with (person / ".calibration.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            calibrate("user", 1.7, tmp_path, lambda *_: pytest.fail("collector started"))
    assert not (person / "pico-simple").exists()
