"""Tracking loss must not accumulate hidden native motion behind a held command."""
from pathlib import Path

import numpy as np
import pytest

from mocap_policy_runtime.integration.native import NativeIK
from mocap_policy_runtime.integration.targets import TargetAdapter

pytestmark = pytest.mark.skipif(
    not (Path(__file__).resolve().parents[1] / "control/build/mocap_tcp_worker").is_file(),
    reason="build mocap_tcp_worker to exercise the actual native controller",
)


def test_one_arm_dropout_preserves_hold_and_recovery_history():
    recovery = []
    for missing_ticks in (1, 100):
        with NativeIK() as ik:
            goals = {side: pose.copy() for side, pose in ik.home_tcp.items()}
            goals["left"][0] += 0.02
            goals["right"][0] += 0.025
            for _ in range(40):
                ik.step(goals)
            held = ik.current_positions[7:].copy()
            for _ in range(missing_ticks):
                result = ik.step({"left": goals["left"]})
                np.testing.assert_array_equal(result[7:], held)
            recovery.append(ik.step({"right": goals["right"]})[7:] - held)
            before = ik.current_positions.copy()
            np.testing.assert_array_equal(ik.step({}), before)
    np.testing.assert_allclose(recovery[0], recovery[1], rtol=0, atol=1e-12)


def test_return_hold_reseeds_worker_after_home_capture():
    recovery = []
    for settled_ticks in (1, 100):
        with NativeIK() as ik:
            home = ik.current_positions.copy()
            home[[0, 7]] += 0.01
            ik.synchronize(home)
            ik.capture_home()
            adapter = TargetAdapter(ik)
            goals = {side: pose.copy() for side, pose in ik.home_tcp.items()}
            goals["right"][0] += 0.025
            for _ in range(40):
                ik.step(goals)
            approached = ik.current_positions.copy()
            for fraction in np.linspace(0.0, 1.0, 101):
                commanded = np.concatenate(
                    ((1.0 - fraction) * approached + fraction * home, np.zeros(40))
                )
                adapter.hold(commanded)
                np.testing.assert_array_equal(ik.current_positions, commanded[:14])
            for _ in range(settled_ticks):
                adapter.hold(commanded)
            for side in ("left", "right"):
                np.testing.assert_allclose(ik.current_tcp[side], ik.home_tcp[side], rtol=0, atol=1e-12)
                np.testing.assert_allclose(ik.current_wrist[side], ik.home_wrist[side], rtol=0, atol=1e-12)
            recovery.append(ik.step(goals))
    np.testing.assert_allclose(recovery[0], recovery[1], rtol=0, atol=1e-12)
