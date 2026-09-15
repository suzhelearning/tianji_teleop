"""Staged gate transitions use synthetic feedback; never load vendor SDKs."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from real_robot.protocol import CommandFrame
from real_robot.safety import SafetyFault
from real_robot.staged_motion import StagedMotionGate

NOW = 10_000_000_000
TICK = 5_000_000  # 200 Hz
ALL = ("arms", "left_hand", "right_hand")
HOME_LEFT = (.5, -.5, -.4, -.3, .3, 0.0, 0.0)
HOME_RIGHT = (-.5, -.5, .4, -.3, -.3, 0.0, 0.0)


def configuration():
    return {
        "rate_hz": 200, "command_timeout_s": .15, "feedback_timeout_s": .15,
        "arms": {"lower_rad": [-2.0]*14, "upper_rad": [2.0]*14,
                 "alignment_rad": .05, "maximum_speed_rad_s": .5, "tracking_error_rad": .14},
        # The hands keep the base class's zero-ramp keys: staged mode must ignore
        # them and go straight to the frozen target from the measured pose.
        "left_hand": {"lower_rad": [-1.0]*20, "upper_rad": [2.0]*20,
                      "alignment_rad": .1, "maximum_speed_rad_s": 1.0, "tracking_error_rad": .4,
                      "enable_zero_ramp_seconds": .5, "enable_zero_tolerance_rad": .15,
                      "enable_zero_timeout_s": 5.0},
        "right_hand": {"lower_rad": [-1.0]*20, "upper_rad": [2.0]*20,
                       "alignment_rad": .1, "maximum_speed_rad_s": 1.0, "tracking_error_rad": .4,
                       "enable_zero_ramp_seconds": .5, "enable_zero_tolerance_rad": .15,
                       "enable_zero_timeout_s": 5.0},
    }


def settings(**overrides):
    result = {"home_left_rad": list(HOME_LEFT), "home_right_rad": list(HOME_RIGHT),
              "maximum_speed_rad_s": .1, "timeout_s": 60.0, "settle_time_s": .2}
    result.update(overrides)
    return result


def frame(value=0.0, flags=7, epoch=7, stamp=NOW, sequence=1):
    return CommandFrame(sequence, stamp, epoch, flags, (value,)*7, (value,)*7,
                        (value,)*20, (value,)*20)


def feedback(enabled=False, stamp=NOW, values=None):
    result = {device: SimpleNamespace(position_rad=(0.0,)*count, received_monotonic_ns=stamp,
                                      healthy=True, enabled=enabled, detail="")
              for device, count in (("arms", 14), ("left_hand", 20), ("right_hand", 20))}
    for device, positions in (values or {}).items():
        result[device].position_rad = tuple(positions)
    return result


def gate(**overrides):
    return StagedMotionGate(configuration(), ALL, settings(**overrides))


def settle(gate, target, measured, now, limit=6000):
    """Drive with feedback exactly following the last command until READY."""
    for _ in range(limit):
        now += TICK
        measured = gate.step(frame(value=target, stamp=now),
                             feedback(enabled=True, stamp=now, values=measured), now)
        if gate.phase == "READY":
            return now, measured
    raise AssertionError(f"never reached READY (phase {gate.phase})")


def held(value=0.0):
    return {device: (value,)*count for device, count in (("arms", 14), ("left_hand", 20), ("right_hand", 20))}


class StagedMotionGateTests(unittest.TestCase):
    def test_tracking_fault_identifies_worst_joint_and_previous_setpoint(self):
        motion = gate()
        motion.arm(frame(value=.5), feedback(), NOW)
        motion.step(frame(value=.5, stamp=NOW + TICK),
                    feedback(enabled=True, stamp=NOW + TICK), NOW + TICK)
        positions = [0.0] * 14
        positions[2] = .20
        positions[9] = -.31
        now = NOW + 2 * TICK
        with self.assertRaises(SafetyFault) as caught:
            motion.step(frame(value=1.0, stamp=now),
                        feedback(enabled=True, stamp=now - 2_000_000,
                                 values={"arms": positions}), now)
        detail = caught.exception.details
        self.assertEqual(detail["kind"], "tracking_error")
        self.assertEqual(detail["device"], "arms")
        self.assertEqual(detail["joint"], "Joint3_R")
        self.assertEqual(detail["joint_index"], 9)
        self.assertAlmostEqual(detail["command_rad"], .0005)
        self.assertAlmostEqual(detail["actual_rad"], -.31)
        self.assertAlmostEqual(detail["error_rad"], .3105)
        self.assertAlmostEqual(detail["absolute_error_rad"], .3105)
        self.assertAlmostEqual(detail["limit_rad"], .14)
        self.assertAlmostEqual(detail["feedback_age_ms"], 2.0)
        self.assertAlmostEqual(detail["step_dt_ms"], 5.0)
        # Diagnostics must not turn the rejected motion into an executable step.
        with self.assertRaises(SafetyFault):
            motion.step(frame(stamp=now + TICK),
                        feedback(enabled=True, stamp=now + TICK), now + TICK)

    def test_hand_following_error_does_not_stop_teleop_or_bypass_feedback_expiry(self):
        motion = gate()
        motion.arm(frame(), feedback(), NOW)
        now, positions = settle(motion, 0.0, held(), NOW)
        motion.start_teleop(frame(stamp=now),
                           feedback(enabled=True, stamp=now, values=positions), now)
        now += TICK
        measured = feedback(enabled=True, stamp=now,
                            values={"left_hand": (.8,) * 20, "right_hand": (.9,) * 20})
        outputs = motion.step(frame(value=1.0, stamp=now), measured, now)
        self.assertEqual(motion.phase, "TELEOP")
        self.assertEqual(outputs["left_hand"], (.005,) * 20)
        self.assertEqual(outputs["right_hand"], (.005,) * 20)
        now += TICK
        measured["left_hand"].received_monotonic_ns = now - 200_000_000
        with self.assertRaises(SafetyFault):
            motion.step(frame(stamp=now), measured, now)

    def test_home_completion_does_not_require_held_fingers_to_close_further(self):
        motion = gate()
        positions = held(.3)
        motion.arm(frame(value=.3), feedback(values=positions), NOW)
        now, positions = settle(motion, .3, positions, NOW)
        now += TICK
        motion.start_teleop(frame(value=.3, stamp=now), feedback(enabled=True, stamp=now, values=positions), now)
        motion.start_homing(frame(value=.3, stamp=now), feedback(enabled=True, stamp=now, values=positions), now)
        for _ in range(2000):
            now += TICK
            # Contact displacement exceeds the former hand following-error limit.
            contact_positions = dict(positions)
            contact_positions["left_hand"] = (.9,) * 20
            contact_positions["right_hand"] = (.9,) * 20
            positions = motion.step(
                frame(value=.3, stamp=now), feedback(enabled=True, stamp=now, values=contact_positions), now)
            if motion.phase == "HOME_REACHED":
                break
        self.assertEqual(motion.phase, "HOME_REACHED")
        self.assertEqual(positions["left_hand"], (.3,) * 20)
        self.assertEqual(positions["right_hand"], (.3,) * 20)

    def test_feedback_fault_at_homing_transition_cannot_be_retried(self):
        motion = gate()
        motion.arm(frame(), feedback(), NOW)
        now, measured = settle(motion, 0.0, held(), NOW)
        now += TICK
        motion.start_teleop(frame(stamp=now), feedback(enabled=True, stamp=now, values=measured), now)
        broken = feedback(enabled=True, stamp=now, values=measured)
        broken["arms"].healthy = False
        with self.assertRaises(SafetyFault):
            motion.start_homing(frame(stamp=now), broken, now)
        with self.assertRaises(SafetyFault):
            motion.start_homing(frame(stamp=now), feedback(enabled=True, stamp=now, values=measured), now)

    def test_large_mismatch_aligns_slowly_instead_of_direct_teleop(self):
        target = frame(value=1.0)
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.check_enable_ready(target, feedback(enabled=False), NOW)  # first Enter authorizes slow alignment
        gate.arm(target, feedback(enabled=False), NOW)
        self.assertTrue(gate.armed)
        self.assertEqual(gate.phase, "ALIGNING")
        self.assertEqual(gate.display_targets["arms"], (1.0,)*14)
        now = NOW + TICK
        output = gate.step(frame(value=1.0, stamp=now), feedback(enabled=True, stamp=now), now)
        self.assertEqual(set(output), set(ALL))
        for device, positions in output.items():
            self.assertAlmostEqual(positions[0], .0005)  # min(.1, per-device) * tick
            self.assertNotEqual(positions[0], 0.0)       # no zero/ramp detour

    def test_frozen_target_ignores_later_input_and_is_copy_safe(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.arm(frame(value=.5), feedback(enabled=False), NOW)
        now = NOW + TICK
        output = gate.step(frame(value=1.5, stamp=now), feedback(enabled=True, stamp=now), now)
        self.assertAlmostEqual(output["arms"][0], .0005)  # toward .5, not toward 1.5
        self.assertEqual(gate.display_targets["arms"], (.5,)*14)
        snapshot = gate.display_targets
        snapshot["arms"] = (0.0,)*14
        snapshot["left_hand"] = (0.0,)*20
        self.assertEqual(gate.display_targets["arms"], (.5,)*14)
        self.assertEqual(gate.display_targets["left_hand"], (.5,)*20)

    def test_no_ready_while_physical_feedback_lags(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        # .12 rad is inside the .14 tracking error but outside the .05 alignment.
        gate.arm(frame(value=.12), feedback(enabled=False), NOW)
        now = NOW
        for _ in range(400):
            now += TICK
            gate.step(frame(value=.12, stamp=now), feedback(enabled=True, stamp=now), now)
            self.assertEqual(gate.phase, "ALIGNING")
        self.assertIsNone(gate.fault)

    def test_ready_requires_settled_feedback_then_holds(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.arm(frame(value=.1), feedback(enabled=False), NOW)
        self.assertEqual(gate.phase, "ALIGNING")
        now, measured = settle(gate, .1, held(), NOW)
        self.assertEqual(gate.phase, "READY")
        self.assertEqual(gate.display_targets["arms"], (.1,)*14)
        now += TICK
        output = gate.step(frame(value=.9, stamp=now),
                           feedback(enabled=True, stamp=now, values=measured), now)
        for device in ALL:
            self.assertEqual(set(output[device]), {.1})  # holds, ignores moved input
        self.assertEqual(gate.phase, "READY")

    def test_second_enter_handoff_is_bounded_and_continuous(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.arm(frame(value=.1), feedback(enabled=False), NOW)
        now, measured = settle(gate, .1, held(), NOW)
        now += TICK
        gate.start_teleop(frame(value=.6, stamp=now),
                          feedback(enabled=True, stamp=now, values=measured), now)
        self.assertEqual(gate.phase, "TELEOP")
        now += TICK
        output = gate.step(frame(value=.6, stamp=now),
                           feedback(enabled=True, stamp=now, values=measured), now)
        self.assertAlmostEqual(output["arms"][0], .1 + .5 * .005)
        self.assertAlmostEqual(output["left_hand"][0], .1 + 1.0 * .005)
        self.assertLess(output["arms"][0], .6)
        self.assertEqual(gate.display_targets["arms"], (.6,)*14)

    def test_homing_locks_supplied_home_and_holds_hands_until_reached(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.arm(frame(value=.1), feedback(enabled=False), NOW)
        now, measured = settle(gate, .1, held(), NOW)
        now += TICK
        gate.start_teleop(frame(value=.1, stamp=now),
                          feedback(enabled=True, stamp=now, values=measured), now)
        now += TICK
        gate.start_homing(frame(value=.6, stamp=now),
                          feedback(enabled=True, stamp=now, values=measured), now)
        home = HOME_LEFT + HOME_RIGHT
        self.assertEqual(gate.phase, "HOMING")
        self.assertEqual(gate.display_targets["arms"], home)
        self.assertEqual(gate.display_targets["left_hand"], (.1,)*20)  # held, not zero/opened
        for _ in range(3000):
            now += TICK
            measured = gate.step(frame(value=.6, stamp=now),
                                 feedback(enabled=True, stamp=now, values=measured), now)
            if gate.phase == "HOME_REACHED":
                break
        self.assertEqual(gate.phase, "HOME_REACHED")
        self.assertEqual(gate.display_targets["arms"], home)
        self.assertEqual(gate.display_targets["left_hand"], (.1,)*20)
        self.assertIsNone(gate.fault)

    def test_stale_frame_epoch_reset_and_feedback_faults_latch(self):
        stale = StagedMotionGate(configuration(), ALL, settings())
        stale.arm(frame(), feedback(enabled=False), NOW)
        with self.assertRaises(SafetyFault):
            stale.step(frame(stamp=NOW - 200_000_000), feedback(enabled=True), NOW)
        self.assertIsNotNone(stale.fault)
        with self.assertRaises(SafetyFault):
            stale.step(frame(stamp=NOW + TICK), feedback(enabled=True, stamp=NOW + TICK), NOW + TICK)

        reset = StagedMotionGate(configuration(), ALL, settings())
        reset.arm(frame(epoch=7), feedback(enabled=False), NOW)
        now = NOW + TICK
        with self.assertRaises(SafetyFault):
            reset.step(frame(epoch=8, stamp=now), feedback(enabled=True, stamp=now), now)
        self.assertIsNotNone(reset.fault)

        unhealthy = StagedMotionGate(configuration(), ALL, settings())
        unhealthy.arm(frame(), feedback(enabled=False), NOW)
        now = NOW + TICK
        with self.assertRaises(SafetyFault):
            unhealthy.step(frame(stamp=now), feedback(enabled=True, stamp=NOW - 200_000_000), now)
        self.assertIsNotNone(unhealthy.fault)

        disabled = StagedMotionGate(configuration(), ALL, settings())
        disabled.arm(frame(), feedback(enabled=False), NOW)
        now = NOW + TICK
        with self.assertRaises(SafetyFault):
            disabled.step(frame(stamp=now), feedback(enabled=False, stamp=now), now)
        self.assertIsNotNone(disabled.fault)

    def test_alignment_timeout_faults_with_detail(self):
        gate = StagedMotionGate(configuration(), ALL, settings(timeout_s=.05))
        gate.arm(frame(value=1.0), feedback(enabled=False), NOW)
        now = NOW
        measured = held()
        with self.assertRaises(SafetyFault) as caught:
            for _ in range(40):
                now += TICK
                measured = gate.step(frame(value=1.0, stamp=now),
                                     feedback(enabled=True, stamp=now, values=measured), now)
        self.assertIn("0.05 s", str(caught.exception))
        self.assertIsNotNone(gate.fault)

    def test_invalid_home_and_premature_transitions_are_rejected(self):
        for home in ((math.nan,) + HOME_LEFT[1:], (3.0,) + HOME_LEFT[1:], HOME_LEFT[:6], None):
            with self.assertRaises(ValueError):
                StagedMotionGate(configuration(), ALL, settings(home_left_rad=home))
        with self.assertRaises(ValueError):
            StagedMotionGate(configuration(), ("left_hand",), settings())

        gate = StagedMotionGate(configuration(), ALL, settings())
        with self.assertRaises(SafetyFault):
            gate.start_teleop(frame(), feedback(enabled=False), NOW)
        with self.assertRaises(SafetyFault):
            gate.start_homing(frame(), feedback(enabled=False), NOW)
        self.assertIsNone(gate.fault)
        gate.arm(frame(), feedback(enabled=False), NOW)
        with self.assertRaises(SafetyFault):
            gate.start_teleop(frame(), feedback(enabled=True), NOW)
        with self.assertRaises(SafetyFault):
            gate.start_homing(frame(), feedback(enabled=True), NOW)



if __name__ == "__main__":
    unittest.main()
