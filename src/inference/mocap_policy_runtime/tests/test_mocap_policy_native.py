"""Tracking loss must not accumulate hidden native motion behind a held command."""

import numpy as np

from mocap_policy_runtime.integration.native import NativeIK
from mocap_policy_runtime.integration.targets import TargetAdapter



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
            adapter.close()
    np.testing.assert_allclose(recovery[0], recovery[1], rtol=0, atol=1e-12)


def test_hand_solver_state_is_per_side_and_reused_only_for_unchanged_source(monkeypatch):
    from mocap_policy_runtime.integration import hand
    from mocap_policy_runtime.types import TargetFrame

    solvers = {}

    class StatefulRetargeter:
        def __init__(self, side):
            self.value = 0.
            self.closed = False
            self.close_count = 0
            solvers[side] = self

        def retarget(self, points):
            assert not self.closed
            self.value += .01
            return np.full(20, self.value)

        def close(self):
            self.closed = True
            self.close_count += 1

    monkeypatch.setattr(hand, "HandRetargeter", StatefulRetargeter)
    with NativeIK() as ik:
        adapter = TargetAdapter(ik)
        try:
            points = np.zeros((21, 3))
            direct = TargetFrame(0., 0, hand_joints={"right": np.full(20, .03)})
            np.testing.assert_allclose(adapter.frame(direct)[34:], .03)
            assert not solvers

            target = TargetFrame(0., 0, hand_keypoints={"right": points})
            np.testing.assert_allclose(adapter.frame(target)[34:], .01)
            np.testing.assert_allclose(adapter.frame(target)[34:], .01)
            assert set(solvers) == {"right"}

            # The source can revise a frame without changing its index.
            points[1, 0] = .05
            np.testing.assert_allclose(adapter.frame(target)[34:], .02)
            target = TargetFrame(.02, 1, hand_keypoints={"left": points, "right": points})
            result = adapter.frame(target)
            np.testing.assert_allclose(result[14:34], .01)
            np.testing.assert_allclose(result[34:], .03)
        finally:
            adapter.close()
            adapter.close()
        assert set(solvers) == {"left", "right"}
        assert all(solver.closed and solver.close_count == 1 for solver in solvers.values())
