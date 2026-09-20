import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.pico_simple_calibration import candidate, height_lengths, mirror_tcp, reflection, load_mirror_contract
from pico_palm_tcp_runtime import quaternion_to_matrix


@pytest.mark.parametrize("height", [float("nan"), float("inf"), 0, -1, 170, True, "1.7"])
def test_height_rejects_units_and_invalid_values(height):
    with pytest.raises(ValueError):
        height_lengths(height)


def test_height_template():
    assert height_lengths(1.7) == {"upper_arm_length_m": 1.7 * .155882,
                                  "forearm_length_m": 1.7 * .152941}


@pytest.mark.parametrize("matrix", [np.eye(3), -np.eye(3), np.zeros((3, 3)),
                                     [[float("nan")]*3]*3])
def test_invalid_reflections(matrix):
    with pytest.raises(ValueError):
        reflection(matrix)


def test_mirror_roundtrip_and_world_composition():
    # Synthetic coordinate conventions, NOT a claim about PICO hardware axes.
    mc, mp = np.diag([1, -1, 1]), np.diag([1, 1, -1])
    world = np.diag([1, -1, 1])
    t = np.array([.02, -.1, .03]); q = np.array([.1, .2, -.3, .9]); q /= np.linalg.norm(q)
    tr, qr = mirror_tcp(t, q, mc, mp)
    tt, qq = mirror_tcp(tr, qr, mc, mp)
    r = quaternion_to_matrix(q); rr = quaternion_to_matrix(qr)
    np.testing.assert_allclose(tt, t, atol=1e-12)
    np.testing.assert_allclose(quaternion_to_matrix(qq), r, atol=1e-12)
    assert np.linalg.det(rr) == pytest.approx(1)
    controller_r = quaternion_to_matrix([.2, .1, .3, .9])
    right_controller_r = world @ controller_r @ mc
    np.testing.assert_allclose(right_controller_r @ rr, world @ controller_r @ r @ mp, atol=1e-12)
    np.testing.assert_allclose(right_controller_r @ tr, world @ controller_r @ t, atol=1e-12)


def test_candidate_preserves_measured_files_and_never_publishes(tmp_path):
    from pico_test_data import synthetic_bundle
    source = synthetic_bundle(tmp_path / "synthetic")
    tcp, wrist = source / "pico_left_palm_tcp.yaml", source / "pico_left_wrist_pivot.yaml"
    before = tcp.read_bytes(), wrist.read_bytes()
    contract = tmp_path / "mirror.json"
    contract.write_text(json.dumps({"schema_version": 1,
        "controller_mirror": np.diag([1, -1, 1]).tolist(),
        "palm_mirror": np.diag([1, -1, 1]).tolist(), "evidence": "synthetic test only"}))
    doc = candidate(tcp, wrist, 1.7, contract)
    assert doc["body"]["left"] == doc["body"]["right"]
    assert doc["wrist"]["left_distance_m"] == doc["wrist"]["right_distance_m"]
    assert doc["runtime_eligible"] is False
    assert doc["candidate_status"] == "review_required"
    assert "quality" not in doc["tcp"]["right"]
    assert "gate_results" not in doc["body"]
    assert before == (tcp.read_bytes(), wrist.read_bytes())
    output = tmp_path / "candidate.json"
    output.write_text("preserve")
    result = subprocess.run([sys.executable, str(ROOT / "scripts/pico_simple_calibration.py"),
        "--left-tcp", str(tcp), "--left-wrist", str(wrist), "--height-m", "1.7",
        "--mirror-contract", str(contract), "--output", str(output)], capture_output=True)
    assert result.returncode == 2
    assert output.read_text() == "preserve"


def test_no_implicit_mirror_axis(tmp_path):
    contract = tmp_path / "mirror.json"
    contract.write_text(json.dumps({"schema_version": 1, "evidence": "no axes"}))
    with pytest.raises(ValueError, match="contract"):
        candidate("missing_tcp", "missing_wrist", 1.7, contract)


def test_model_requires_explicit_selection():
    with pytest.raises(ValueError, match="exactly one"):
        load_mirror_contract()
    with pytest.raises(ValueError, match="exactly one"):
        load_mirror_contract("unused.json", "symmetric_local_y")
    with pytest.raises(ValueError, match="unknown"):
        load_mirror_contract(mirror_model="guess_hardware_axes")


def test_named_model_uses_left_only_not_zj_right_fit(tmp_path):
    import shutil
    from pico_test_data import synthetic_bundle
    source = synthetic_bundle(tmp_path / "synthetic")
    tcp, wrist = [tmp_path / name for name in ("pico_left_palm_tcp.yaml", "pico_left_wrist_pivot.yaml")]
    for p in (tcp, wrist):
        shutil.copyfile(source / p.name, p)
    # No right-side files exist in this isolated input directory.
    doc = candidate(tcp, wrist, 1.7, mirror_model="symmetric_local_y")
    l, r = doc["tcp"]["left"], doc["tcp"]["right"]
    m = np.diag([1, -1, 1])
    np.testing.assert_allclose(r["translation_m"], m @ l["translation_m"], atol=1e-12)
    np.testing.assert_allclose(quaternion_to_matrix(r["quaternion_xyzw"]),
                               m @ quaternion_to_matrix(l["quaternion_xyzw"]) @ m, atol=1e-12)
    assert np.linalg.norm(r["translation_m"]) == pytest.approx(np.linalg.norm(l["translation_m"]))
    assert doc["body"]["left"] == doc["body"]["right"]
    assert doc["wrist"]["left_distance_m"] == doc["wrist"]["right_distance_m"]
    assert doc["mirror_model"] == "symmetric_local_y"
    assert doc["runtime_eligible"] is False
