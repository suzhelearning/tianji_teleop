"""Staged gate transitions use synthetic feedback; never load vendor SDKs."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace
import unittest

from tianji_controller.protocol import CommandFrame
from tianji_controller.safety import SafetyFault
from tianji_controller.staged_motion import StagedMotionGate

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
        command = motion.step(frame(value=.5, stamp=NOW + TICK),
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
        self.assertEqual(detail["command_rad"], command["arms"][9])
        self.assertAlmostEqual(detail["actual_rad"], -.31)
        self.assertAlmostEqual(detail["error_rad"], command["arms"][9] + .31)
        self.assertAlmostEqual(detail["absolute_error_rad"], command["arms"][9] + .31)
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
        for _ in range(4000):
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
        now = NOW
        previous = held()
        moved = False
        for _ in range(120):
            now += TICK
            output = gate.step(frame(value=1.0, stamp=now),
                               feedback(enabled=True, stamp=now, values=previous), now)
            self.assertEqual(set(output), set(ALL))
            for device in ALL:
                step = output[device][0] - previous[device][0]
                self.assertGreaterEqual(step, 0)
                self.assertLessEqual(step, .1 * TICK / 1e9 + 1e-12)
                moved |= step > 0
            previous = output
        self.assertTrue(moved)

    def test_frozen_target_ignores_later_input_and_is_copy_safe(self):
        gate = StagedMotionGate(configuration(), ALL, settings())
        gate.arm(frame(value=.5), feedback(enabled=False), NOW)
        now = NOW + TICK
        output = gate.step(frame(value=1.5, stamp=now), feedback(enabled=True, stamp=now), now)
        # New live input cannot alter the frozen trajectory or its endpoint.
        now, output = settle(gate, 1.5, output, now)
        self.assertEqual(output["arms"], (.5,) * 14)
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

    def test_infeasible_approach_is_rejected_before_motion_and_latches_fault(self):
        motion = gate(timeout_s=.05)
        with self.assertRaisesRegex(SafetyFault, "requires at least"):
            motion.arm(frame(value=1.0), feedback(enabled=False), NOW)
        self.assertIsNotNone(motion.fault)
        self.assertEqual(motion._last_commands, held())

    def test_feasible_approach_still_faults_if_actual_feedback_never_settles(self):
        motion = gate(timeout_s=3)
        motion.arm(frame(value=.12), feedback(enabled=False), NOW)
        now = NOW
        with self.assertRaisesRegex(SafetyFault, "did not settle"):
            for _ in range(700):
                now += TICK
                motion.step(frame(value=.12, stamp=now), feedback(enabled=True, stamp=now), now)
        self.assertIsNotNone(motion.fault)

    def test_nonuniform_ticks_preserve_wall_time_velocity_and_acceleration_bounds(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(value=.2), feedback(), NOW)
        now, command = NOW, held()
        old_velocity, old_dt = 0.0, 0.0
        for index in range(2000):
            dt_ns = (2_000_000, 13_000_000, 5_000_000, 9_000_000, 3_000_000)[index % 5]
            dt = dt_ns / 1e9
            now += dt_ns
            motion.paused = 100 <= index < 180 or 350 <= index < 430
            previous = command
            command = motion.step(frame(value=.2, stamp=now),
                                  feedback(enabled=True, stamp=now, values=previous), now)
            velocity = (command["arms"][0] - previous["arms"][0]) / dt
            self.assertLessEqual(abs(velocity), .1 + 1e-10)
            # Sample-average velocities live at adjacent interval midpoints.
            self.assertLessEqual(abs(velocity - old_velocity) / ((dt + old_dt) / 2),
                                 .2 + 1e-8)
            old_velocity, old_dt = velocity, dt
            if motion.phase == "READY":
                break
        self.assertEqual(motion.phase, "READY")
        self.assertEqual(command["arms"], (.2,) * 14)

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

    def test_pause_brakes_and_early_resume_waits_for_measured_rest_without_jump(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(value=.3), feedback(), NOW)
        now, command = NOW, held()
        for _ in range(200):
            now += TICK
            command = motion.step(frame(value=.3, stamp=now),
                                  feedback(enabled=True, stamp=now, values=command), now)
        self.assertFalse(motion.staged_stopped)
        motion.paused = True
        now += TICK
        command = motion.step(frame(value=.3, stamp=now),
                              feedback(enabled=True, stamp=now, values=command), now)
        self.assertFalse(motion.staged_stopped)
        motion.paused = False  # Cannot cancel the committed brake.
        for _ in range(200):
            now += TICK
            command = motion.step(frame(value=.3, stamp=now),
                                  feedback(enabled=True, stamp=now, values=command), now)
            if motion.staged_stopped:
                break
        self.assertTrue(motion.staged_stopped)
        stopped = command
        self.assertLess(stopped["arms"][0], .3)
        # Moving feedback inside pose tolerance is still not rest.
        for index in range(30):
            now += TICK
            moving = {d: tuple(q + (.01 if index % 2 else -.01) for q in qs)
                      for d, qs in stopped.items()}
            command = motion.step(frame(value=.3, stamp=now),
                                  feedback(enabled=True, stamp=now, values=moving), now)
            self.assertEqual(command, stopped)
            self.assertNotEqual(motion.phase, "READY")
        # A stationary encoder offset must NOT become the new command seed.
        lagged = {d: tuple(q + .02 for q in qs) for d, qs in stopped.items()}
        for _ in range(40):
            now += TICK
            command = motion.step(frame(value=.3, stamp=now),
                                  feedback(enabled=True, stamp=now, values=lagged), now)
            if command != stopped:
                break
        self.assertGreater(command["arms"][0], stopped["arms"][0])
        self.assertLess(command["arms"][0] - stopped["arms"][0], .1 * TICK / 1e9)
        self.assertEqual(motion.display_targets["arms"], (.3,) * 14)
        now, command = settle(motion, .3, command, now)
        self.assertEqual(command["arms"], (.3,) * 14)

    def test_repeated_pauses_do_not_consume_timeout_or_bypass_watchdogs(self):
        motion = gate(timeout_s=3, settle_time_s=.05)
        motion.arm(frame(value=.02), feedback(), NOW)
        now, command = NOW, held()
        for _ in range(3):
            for _ in range(35):
                now += TICK
                command = motion.step(frame(value=.02, stamp=now),
                                      feedback(enabled=True, stamp=now, values=command), now)
            motion.paused = True
            for _ in range(700):
                now += TICK
                command = motion.step(frame(value=.02, stamp=now),
                                      feedback(enabled=True, stamp=now, values=command), now)
                self.assertNotEqual(motion.phase, "READY")
            self.assertTrue(motion.staged_stopped)
            motion.paused = False
        now, command = settle(motion, .02, command, now)
        self.assertEqual(command["arms"], (.02,) * 14)
        motion.paused = True
        now += 200_000_000
        with self.assertRaises(SafetyFault):
            motion.step(frame(value=.02, stamp=now),
                        feedback(enabled=True, stamp=now, values=command), now)
        self.assertIsNotNone(motion.fault)

    def test_initial_approach_waits_for_stationary_feedback_and_rejects_cached_rest(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(value=.1), feedback(), NOW)
        now = NOW
        for index in range(50):
            now += TICK
            command = motion.step(frame(value=.1, stamp=now),
                                  feedback(enabled=True, stamp=now, values=held(index * .001)), now)
            self.assertEqual(command, held())
        cached_stamp = now
        for _ in range(20):
            now += TICK
            command = motion.step(frame(value=.1, stamp=now),
                                  feedback(enabled=True, stamp=cached_stamp, values=held(.049)), now)
            self.assertEqual(command, held())
            self.assertNotEqual(motion.phase, "READY")

    def test_homing_refuses_moving_teleop_but_allows_verified_stationary_handoff(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(), feedback(), NOW)
        now, command = settle(motion, 0.0, held(), NOW)
        motion.start_teleop(frame(stamp=now), feedback(enabled=True, stamp=now, values=command), now)
        now += TICK
        command = motion.step(frame(value=.02, stamp=now),
                              feedback(enabled=True, stamp=now, values=command), now)
        with self.assertRaisesRegex(SafetyFault, "stationary"):
            motion.start_homing(frame(value=.02, stamp=now),
                                feedback(enabled=True, stamp=now, values=command), now)
        self.assertEqual(motion.phase, "TELEOP")
        self.assertIsNone(motion.fault)
        for _ in range(50):
            now += TICK
            command = motion.step(frame(value=.02, stamp=now),
                                  feedback(enabled=True, stamp=now, values=command), now)
        motion.start_homing(frame(value=.02, stamp=now),
                            feedback(enabled=True, stamp=now, values=command), now)
        self.assertEqual(motion.phase, "HOMING")

    def test_moving_finger_targets_do_not_block_stationary_arm_home(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(), feedback(), NOW)
        now, command = settle(motion, 0.0, held(), NOW)
        motion.start_teleop(frame(stamp=now), feedback(enabled=True, stamp=now, values=command), now)
        now += TICK
        source = CommandFrame(1, now, 7, 7, (0.0,) * 7, (0.0,) * 7, (.8,) * 20, (.6,) * 20)
        command = motion.step(source, feedback(enabled=True, stamp=now, values=command), now)
        self.assertGreater(command["left_hand"][0], 0)
        self.assertLess(command["left_hand"][0], .8)
        motion.start_homing(source, feedback(enabled=True, stamp=now, values=command), now)
        self.assertEqual(motion.phase, "HOMING")
        self.assertEqual(motion.display_targets["left_hand"], command["left_hand"])
        self.assertEqual(motion.display_targets["right_hand"], command["right_hand"])

    def test_reset_cannot_instantaneously_stop_a_moving_trajectory(self):
        motion = gate(settle_time_s=.05)
        motion.arm(frame(value=.3), feedback(), NOW)
        now, command = NOW, held()
        for _ in range(80):
            now += TICK
            command = motion.step(frame(value=.3, stamp=now),
                                  feedback(enabled=True, stamp=now, values=command), now)
        self.assertFalse(motion.staged_stopped)
        with self.assertRaisesRegex(SafetyFault, "command velocity"):
            motion.reset_staged_motion()
        self.assertIsNotNone(motion.fault)

    def test_invalid_acceleration_is_rejected_before_enable(self):
        for acceleration in (0, -1, math.inf, math.nan, True):
            with self.subTest(acceleration=acceleration), self.assertRaises(ValueError):
                gate(maximum_acceleration_rad_s2=acceleration)



if __name__ == "__main__":
    unittest.main()
