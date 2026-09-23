import importlib.util
from pathlib import Path
import struct
import sys
import zlib

import numpy as np
import pytest
import yaml

from tianji_runtime import workspace

ROOT = workspace()
sys.path.insert(0, str(ROOT / "src/teleop_inputs/pico_controller/scripts"))
from calibrate_pico_simple import build_bundle
from pico_palm_orientation_core import OrientationSolution
from pico_palm_tcp_calibrator import build_tcp_artifact
spec = importlib.util.spec_from_file_location(
    "trace_audit", Path(__file__).resolve().parents[1] / "scripts/audit_shared_root_trace.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def setup(tmp_path):
    directory = tmp_path / "calibration"
    directory.mkdir()
    orientation = OrientationSolution(
        rotation=np.eye(3), covariance=np.eye(3) * 1e-5,
        sample_count=120, tracking_epoch=1, orientation_rms_rad=.01,
        correction_angle_rad=.02)
    tcp = build_tcp_artifact(
        side="left", source_topic="/pico/pose/left_hand",
        translation=np.array([.1, -.02, .03]), rotation=np.eye(3),
        sample_count=20, position_rms=.003, calibration_revision=1,
        sample_matrix_rank=6, sample_matrix_condition=12.,
        orientation_solution=orientation)
    tcp["source_recording"] = "synthetic_offline_fixture"
    (directory / "pico_left_palm_tcp.yaml").write_text(yaml.safe_dump(tcp))
    build_bundle(directory, 1.62, height_wrist=True)
    return directory, tmp_path / "input.tjvr"


def write_trace(directory, trace, mutation=None):
    lengths = module.calibration(directory)["expected_segments_m"]
    records = []
    for frame in range(3):
        packet = bytearray(656)
        seq = frame+1 if mutation != "duplicate" else 1
        struct.pack_into("<4sHHQQqqI", packet, 0, b"TJVR", 4, 656, seq, 1,
                         seq*10000000, 1000000000+frame*10000000, 0xff)
        for offset in (44, 100):
            struct.pack_into("<7d", packet, offset, 0, 0, 0, 1, 0, 0, 0)
        points = []
        for side in range(2):
            x = 0
            y = .2 if side == 0 else -.2
            points.append([x, y, 1.121])
            for i, length in enumerate(lengths[side]):
                x += length + (.07 if mutation == "asymmetric" and side == 0 and i == 1 else 0)
                points.append([x, y, 1.121])
        points = np.asarray(points)
        if mutation == "origin":
            points[:, 2] += .1
        struct.pack_into("<24d", packet, 204, *points.flatten())
        struct.pack_into("<32d", packet, 396, *([1, 0, 0, 0]*8))
        struct.pack_into("<I", packet, 652, zlib.crc32(packet[:652]))
        if mutation == "crc":
            packet[90] ^= 1
        records.append(struct.pack("<Q", frame*10000000) + packet)
    trace.write_bytes(struct.pack("<4sHHQ", b"TJVT", 1, 656, 3)+b"".join(records))


def test_consistent_not_hardware_acceptance(setup):
    directory, trace = setup
    write_trace(directory, trace)
    result = module.audit(trace, directory)
    assert result["geometry_consistent"]
    assert not result["phase_a_accepted"] and not result["publisher_attestation"]
    assert result["actual"] == "unavailable"
    assert result["geometry_samples"] == 3


@pytest.mark.parametrize("mutation,error", [("asymmetric","effective_geometry_mismatch"),
    ("origin","origin_mismatch"), ("crc","crc"), ("duplicate","duplicate")])
def test_rejects_mismatched_data(setup, mutation, error):
    directory, trace = setup
    write_trace(directory, trace, mutation)
    result = module.audit(trace, directory)
    assert not result["geometry_consistent"]
    assert result["errors"][error] > 0


def test_truncated_trace(setup):
    directory, trace = setup
    write_trace(directory, trace)
    trace.write_bytes(trace.read_bytes()[:-1])
    with pytest.raises(ValueError):
        module.audit(trace, directory)
