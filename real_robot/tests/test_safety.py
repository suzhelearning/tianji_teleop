"""Safety transitions use synthetic feedback; never load vendor SDKs."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from real_robot.protocol import CommandFrame
from real_robot.safety import MotionGate, SafetyFault

NOW = 10_000_000_000


def configuration():
    return {
        "rate_hz": 200, "command_timeout_s": .15, "feedback_timeout_s": .15,
        "arms": {"lower_rad": [-2.0]*14, "upper_rad": [2.0]*14,
                 "alignment_rad": .05, "maximum_speed_rad_s": .5, "tracking_error_rad": .14},
        "left_hand": {"lower_rad": [-1.0]*20, "upper_rad": [2.0]*20,
                      "alignment_rad": .1, "maximum_speed_rad_s": 1.0, "tracking_error_rad": .4},
        "right_hand": {"lower_rad": [-1.0]*20, "upper_rad": [2.0]*20,
                       "alignment_rad": .1, "maximum_speed_rad_s": 1.0, "tracking_error_rad": .4},
    }


def zero_ramp_configuration(ramp_seconds=.5, tolerance=.05, timeout=2.0):
    config = configuration()
    for name in ("left_hand", "right_hand"):
        config[name] = dict(config[name], enable_zero_ramp_seconds=ramp_seconds,
                            enable_zero_tolerance_rad=tolerance, enable_zero_timeout_s=timeout)
    return config


def frame(value=0.0, flags=7, epoch=7, stamp=NOW, sequence=1, left_hand=None, right_hand=None):
    return CommandFrame(sequence, stamp, epoch, flags, (value,)*7, (value,)*7,
                        tuple(left_hand) if left_hand is not None else (value,)*20,
                        tuple(right_hand) if right_hand is not None else (value,)*20)


def feedback(enabled=False, stamp=NOW, value=0.0):
    return {device: SimpleNamespace(position_rad=(value,)*count, received_monotonic_ns=stamp,
                                    healthy=True, enabled=enabled, detail="")
            for device, count in (("arms", 14), ("left_hand", 20), ("right_hand", 20))}


class MotionGateTests(unittest.TestCase):
    def test_cannot_send_before_explicit_arm(self):
        gate = MotionGate(configuration(), ("arms", "right_hand"))
        with self.assertRaises(SafetyFault):
            gate.step(frame(), feedback(), NOW)

    def test_startup_must_match_measured_pose(self):
        gate = MotionGate(configuration(), ("arms", "right_hand"))
        with self.assertRaises(SafetyFault):
            gate.arm(frame(.3), feedback(), NOW)
        self.assertFalse(gate.armed)

    def test_slew_starts_at_feedback_not_zero(self):
        gate = MotionGate(configuration(), ("right_hand",))
        gate.arm(frame(.5), feedback(value=.5), NOW)
        commands = gate.step(frame(1.0, stamp=NOW+5_000_000, sequence=2),
                             feedback(enabled=True, value=.5, stamp=NOW+5_000_000), NOW+5_000_000)
        self.assertEqual(set(commands), {"right_hand"})
        self.assertAlmostEqual(commands["right_hand"][0], .505)

    def test_stale_or_disabled_selected_input_latches_fault(self):
        for invalid in (frame(flags=1, stamp=NOW+5_000_000), frame(stamp=NOW-200_000_000)):
            gate = MotionGate(configuration(), ("right_hand",))
            gate.arm(frame(), feedback(), NOW)
            with self.assertRaises(SafetyFault):
                gate.step(invalid, feedback(enabled=True), NOW+5_000_000)
            with self.assertRaises(SafetyFault):
                gate.step(frame(stamp=NOW+10_000_000), feedback(enabled=True), NOW+10_000_000)

    def test_left_only_input_cannot_enable_right_hardware(self):
        gate = MotionGate(configuration(), ("right_hand",))
        with self.assertRaises(SafetyFault):
            gate.arm(frame(flags=4), feedback(), NOW)

    def test_epoch_reset_faults_arms_but_not_independent_right_hand(self):
        arms = MotionGate(configuration(), ("arms",))
        arms.arm(frame(), feedback(), NOW)
        with self.assertRaises(SafetyFault):
            arms.step(frame(epoch=8, stamp=NOW+5_000_000), feedback(enabled=True), NOW+5_000_000)
        hand = MotionGate(configuration(), ("right_hand",))
        hand.arm(frame(), feedback(), NOW)
        self.assertIn("right_hand", hand.step(frame(epoch=8, stamp=NOW+5_000_000),
                                               feedback(enabled=True), NOW+5_000_000))

    def test_invalid_target_or_feedback_never_reaches_output(self):
        for target, measured in ((frame(math.nan), feedback()), (frame(9), feedback()),
                                 (frame(), feedback(stamp=NOW-200_000_000)),
                                 (frame(), feedback(value=9))):
            gate = MotionGate(configuration(), ("arms",))
            with self.assertRaises(SafetyFault):
                gate.arm(target, measured, NOW)

    def test_feedback_accepts_bound_margin_but_rejects_further_excursion(self):
        gate = MotionGate(configuration(), ("arms",))
        gate.check_feedback("arms", feedback(value=2.0 + 0.01)["arms"], NOW)
        with self.assertRaises(SafetyFault):
            gate.check_feedback("arms", feedback(value=2.0 + 0.010001)["arms"], NOW)

    def test_tracking_error_and_servo_disable_stop_active_session(self):
        for measured in (feedback(enabled=True, value=.5), feedback(enabled=False)):
            gate = MotionGate(configuration(), ("arms",))
            gate.arm(frame(), feedback(), NOW)
            with self.assertRaises(SafetyFault):
                gate.step(frame(stamp=NOW+5_000_000), measured, NOW+5_000_000)

    def test_clock_stall_does_not_allow_large_catchup_step(self):
        gate = MotionGate(configuration(), ("right_hand",))
        gate.arm(frame(), feedback(), NOW)
        now = NOW + 100_000_000
        output = gate.step(frame(1, stamp=now), feedback(enabled=True, stamp=now), now)
        self.assertAlmostEqual(output["right_hand"][0], .005)

    def test_all_requires_both_hand_sources_before_arming(self):
        for flags in (3, 5):
            gate = MotionGate(configuration(), ("arms", "left_hand", "right_hand"))
            with self.assertRaises(SafetyFault):
                gate.arm(frame(flags=flags), feedback(), NOW)
            self.assertFalse(gate.armed)

    def test_left_only_uses_left_targets_without_right_readiness(self):
        gate = MotionGate(configuration(), ("left_hand",))
        gate.arm(frame(flags=4), feedback(), NOW)
        now = NOW + 5_000_000
        output = gate.step(frame(flags=4, stamp=now, left_hand=(.4,)*20, right_hand=(-.4,)*20),
                           feedback(enabled=True, stamp=now), now)
        self.assertEqual(set(output), {"left_hand"})
        self.assertAlmostEqual(output["left_hand"][0], .005)

    def test_zero_ramp_enable_needs_no_pose_match_and_starts_at_measured(self):
        gate = MotionGate(zero_ramp_configuration(), ("right_hand",))
        gate.arm(frame(value=.5), feedback(value=.5), NOW)
        self.assertTrue(gate.armed)
        output = gate.step(frame(value=.5), feedback(enabled=True, value=.5), NOW + 5_000_000)
        # The setpoint walks down from the measured pose instead of jumping to 0.
        self.assertAlmostEqual(output["right_hand"][0], .495)

    def test_bimanual_zero_ramps_have_independent_start_times(self):
        gate = MotionGate(zero_ramp_configuration(), ("left_hand", "right_hand"))
        measured = feedback(value=.5)
        measured["left_hand"].position_rad = (.2,) * 20
        gate.arm(frame(value=.6), measured, NOW)
        positions = {"left_hand": (.2,) * 20, "right_hand": (.5,) * 20}
        zero_at = {}
        ramp_samples = {}
        for tick in range(1, 260):
            now = NOW + tick * 5_000_000
            samples = feedback(enabled=True, stamp=now)
            for name, values in positions.items():
                samples[name].position_rad = values
            output = gate.step(frame(value=.6, stamp=now), samples, now)
            for name, values in output.items():
                self.assertLessEqual(max(abs(a-b) for a, b in zip(values, positions[name])), .006001)
                if values == (0.0,) * 20:
                    zero_at[name] = tick
                if name in zero_at and tick == zero_at[name] + 50:
                    ramp_samples[name] = values[0]
            positions = output
        self.assertLess(zero_at["left_hand"], zero_at["right_hand"])
        self.assertEqual(set(ramp_samples), {"left_hand", "right_hand"})
        for value in ramp_samples.values():
            self.assertAlmostEqual(value, .3)
        self.assertEqual(positions["left_hand"], (.6,) * 20)
        self.assertEqual(positions["right_hand"], (.6,) * 20)

    def test_hand_zero_return_remains_speed_limited_without_tracking_shutdown(self):
        gate = MotionGate(zero_ramp_configuration(), ("right_hand",))
        gate.arm(frame(value=.6), feedback(value=.5), NOW)
        previous = .5
        for tick in range(1, 150):
            now = NOW + tick * 5_000_000
            output = gate.step(frame(value=.6, stamp=now),
                               feedback(enabled=True, value=.5, stamp=now), now)
            commanded = output["right_hand"][0]
            self.assertAlmostEqual(commanded, max(0.0, previous - .005))
            previous = commanded
        self.assertEqual(output["right_hand"], (0.0,) * 20)

    def test_zero_return_timeout_with_small_residual(self):
        gate = MotionGate(zero_ramp_configuration(timeout=.2), ("right_hand",))
        gate.arm(frame(value=.6), feedback(value=.1), NOW)
        with self.assertRaises(SafetyFault):
            for tick in range(1, 60):
                now = NOW + tick * 5_000_000
                gate.step(frame(value=.6, stamp=now),
                          feedback(enabled=True, value=.1, stamp=now), now)
        self.assertIsNotNone(gate.fault)

    def test_enable_ramp_preserves_stale_input_guard(self):
        gate = MotionGate(zero_ramp_configuration(), ("right_hand",))
        gate.arm(frame(value=.6), feedback(), NOW)
        gate.step(frame(value=.6, stamp=NOW+5_000_000),
                  feedback(enabled=True), NOW+5_000_000)
        with self.assertRaises(SafetyFault):
            gate.step(frame(value=.6), feedback(enabled=True, stamp=NOW+200_000_000),
                      NOW+200_000_000)

    def test_stale_left_feedback_stops_the_bimanual_session(self):
        gate = MotionGate(configuration(), ("left_hand", "right_hand"))
        gate.arm(frame(), feedback(), NOW)
        now = NOW + 5_000_000
        measured = feedback(enabled=True, stamp=now)
        measured["left_hand"].received_monotonic_ns = NOW - 200_000_000
        with self.assertRaises(SafetyFault):
            gate.step(frame(stamp=now), measured, now)
        with self.assertRaises(SafetyFault):
            gate.step(frame(stamp=now+1), feedback(enabled=True, stamp=now+1), now+1)


if __name__ == "__main__":
    unittest.main()
