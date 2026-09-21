"""Pure loader checks with ROS imports, no rclpy.init, nodes or devices."""
from pathlib import Path
import shutil
import sys
import pytest

pytest.importorskip("rclpy", reason="requires a ROS-compatible Python ABI")

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from calibrate_pico_simple import build_bundle
from pico_simple_artifact import LEFT_TCP, LEFT_WRIST
from pico_palm_skeleton_filter_node import (
    load_arm_geometry_artifact, load_tcp_calibration_revision,
    load_wrist_pivot_artifact, file_sha256,
)


@pytest.mark.parametrize("height_wrist", [False, True])
def test_actual_m0_loaders_accept_simple_bundle(tmp_path, height_wrist):
    from pico_test_data import synthetic_bundle
    source = synthetic_bundle(tmp_path / "synthetic")
    for name in ((LEFT_TCP,) if height_wrist else (LEFT_TCP, LEFT_WRIST)):
        shutil.copyfile(source / name, tmp_path / name)
    build_bundle(tmp_path, 1.7, height_wrist=height_wrist)
    distances = []
    for side in ("left", "right"):
        tcp = tmp_path / f"pico_{side}_palm_tcp.yaml"
        wrist = tmp_path / f"pico_{side}_wrist_pivot.yaml"
        geometry = tmp_path / f"pico_{side}_arm_geometry.yaml"
        revision = load_tcp_calibration_revision(tcp, side)
        g = load_arm_geometry_artifact(geometry, expected_side=side, expected_tcp_revision=revision,
            expected_tcp_sha256=file_sha256(tcp), expected_wrist_pivot_sha256=file_sha256(wrist),
            tcp_path=tcp, wrist_path=wrist)
        assert g.upper_arm_length_m == 1.7 * .155882
        assert g.forearm_length_m == 1.7 * .152941
        distance, status = load_wrist_pivot_artifact(wrist, side)
        assert distance is not None, status
        distances.append(distance)
    assert distances[0] == distances[1]
    if height_wrist:
        assert distances[0] == pytest.approx(1.7 * .037037)
