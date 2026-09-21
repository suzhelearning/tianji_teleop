import json
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from pico_m0_comparison_report import REQUIRED_STREAMS, build_report  # noqa: E402
from pico_palm_skeleton_filter_core import SideCalibration, correct_side  # noqa: E402


def _pose_frame(offset=0.0):
    pose = np.zeros((24, 7), dtype=np.float64)
    pose[:, 6] = 1.0
    pose[:, 0] = np.linspace(0.0, 0.23, 24) + offset
    pose[16, :3] = (0.0 + offset, 0.2, 1.4)
    pose[18, :3] = (0.2845 + offset, 0.2, 1.4)
    pose[20, :3] = (0.5085 + offset, 0.2, 1.4)
    pose[22, :3] = (0.6085 + offset, 0.2, 1.4)
    pose[17, :3] = (0.0 + offset, -0.2, 1.4)
    pose[19, :3] = (0.2845 + offset, -0.2, 1.4)
    pose[21, :3] = (0.5085 + offset, -0.2, 1.4)
    pose[23, :3] = (0.6085 + offset, -0.2, 1.4)
    return pose


def _write_capture(path, *, status_count=5, context_change=False, epoch_change=False):
    count = 5
    stamps = np.arange(count, dtype=np.int64) * 10_000_000 + 1_000_000_000
    raw = np.stack([_pose_frame(0.001 * index) for index in range(count)])
    corrected = raw.copy()
    palms = np.zeros((count, 7), dtype=np.float64)
    palms[:, 6] = 1.0
    palms[:, :3] = raw[:, 22, :3]
    right_palms = palms.copy()
    right_palms[:, :3] = raw[:, 23, :3]
    calibration = SideCalibration(
        upper_length_m=0.2845,
        forearm_length_m=0.2240,
        palm_to_wrist_offset_m=np.zeros(3, dtype=np.float64),
        palm_to_wrist_quat_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
    )
    previous_elbow = None
    for index in range(count):
        result = correct_side(
            raw[index, :, :3],
            raw[index, :, 3:7],
            palms[index, :3],
            palms[index, 3:7],
            calibration,
            "left",
            previous_elbow_position=previous_elbow,
            ratio_max_stretch=0.995,
            wrist_to_palm_distance_m=0.1,
        )
        corrected[index, :, :3] = result.positions
        corrected[index, :, 3:7] = result.orientations_xyzw
        previous_elbow = result.positions[18].copy()
    statuses = []
    for index in range(status_count):
        statuses.append(
            json.dumps(
                {
                    "source_stamp_ns": int(stamps[min(index, count - 1)]),
                    "ratio_max_stretch": 0.995,
                    "tracking_epoch": 1 if not epoch_change or index < 3 else 2,
                    "tracking_epoch_source": "tcp_connection",
                    "stream_valid": True,
                    "left": {
                        "corrected": True,
                        "fallback_reason": "",
                        "reach_clamped": False,
                        "time_skew_ms": 0.0,
                        "geometry_source": "quick_arm_artifact",
                        "geometry_revision": 10 if not context_change or index < 3 else 11,
                        "upper_arm_length_m": 0.2845,
                        "forearm_length_m": 0.2240,
                        "wrist_to_palm_distance_m": 0.1,
                    },
                    "right": {
                        "corrected": False,
                        "fallback_reason": "raw_baseline",
                        "reach_clamped": False,
                        "time_skew_ms": 0.0,
                        "geometry_source": "raw_smpl_baseline",
                        "geometry_revision": 0,
                        "upper_arm_length_m": None,
                        "forearm_length_m": None,
                        "wrist_to_palm_distance_m": 0.1,
                    },
                },
                sort_keys=True,
            )
        )
    np.savez_compressed(
        path,
        raw_pose=raw,
        raw_timestamp_ns=stamps,
        corrected_pose=corrected,
        corrected_timestamp_ns=stamps,
        left_palm_pose=palms,
        left_palm_timestamp_ns=stamps,
        right_palm_pose=right_palms,
        right_palm_timestamp_ns=stamps,
        status_json=np.asarray(statuses),
        tracking_epoch=np.ones(count, dtype=np.uint64),
        tracking_epoch_status_json=np.asarray(
            [json.dumps({"tracking_epoch": 1, "tracking_epoch_source": "tcp_connection"})]
            * count
        ),
    )


def test_required_stream_contract_is_complete():
    assert REQUIRED_STREAMS == (
        "/pico/smpl_raw",
        "/pico/palm_left",
        "/pico/palm_right",
        "/pico/smpl_palm_corrected",
        "/pico/smpl_palm_corrected/status",
        "/pico/tracking_epoch",
        "/pico/tracking_epoch/status",
    )


def test_report_rejects_status_length_mismatch(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture, status_count=4)
    with pytest.raises(ValueError, match="stream_length_mismatch"):
        build_report(capture)


def test_one_side_raw_is_allowed_but_reported(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture)
    report = build_report(capture)
    assert report["valid"]
    assert report["left"]["geometry_source"] == "quick_arm_artifact"
    assert report["left"]["geometry_revision"] == 10
    assert report["right"]["geometry_source"] == "raw_smpl_baseline"
    assert report["right"]["geometry_revision"] == 0


def test_report_rejects_geometry_context_transition(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture, context_change=True)
    with pytest.raises(ValueError, match="geometry_context_changed:left"):
        build_report(capture)


def test_report_rejects_tracking_epoch_transition(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture, epoch_change=True)
    with pytest.raises(ValueError, match="tracking_epoch_changed"):
        build_report(capture)


def test_report_is_deterministic_and_finite(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture)
    first = build_report(capture)
    second = build_report(capture)
    assert first == second
    assert first["deterministic_replay_max_difference"] == 0.0
    assert first["recorded_replay_max_difference"] == 0.0
    assert first["output_frequency_hz"] == pytest.approx(100.0)
    assert first["maximum_gap_s"] == pytest.approx(0.01)
    assert first["left"]["corrected_coverage"] == pytest.approx(1.0)


def test_report_rejects_recorded_output_that_does_not_match_replay(tmp_path):
    capture = tmp_path / "capture.npz"
    _write_capture(capture)
    with np.load(capture, allow_pickle=False) as archive:
        arrays = {name: np.asarray(archive[name]) for name in archive.files}
    arrays["corrected_pose"] = arrays["corrected_pose"].copy()
    arrays["corrected_pose"][2, 18, 0] += 1.0e-4
    np.savez_compressed(capture, **arrays)
    with pytest.raises(ValueError, match="recorded_replay_mismatch"):
        build_report(capture)
