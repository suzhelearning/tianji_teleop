"""Numerical contracts of fixed-path command motion, without hardware."""
import math
import unittest

from real_robot.approach_trajectory import ApproachTrajectory


class ApproachTrajectoryTests(unittest.TestCase):
    def test_signed_joints_share_progress_and_finish_together_with_bounded_derivatives(self):
        start = (0.0,) * 14
        target = (.4, -.2, .1, 0.0, -.35, .3, .05) * 2
        speed, acceleration, dt = .1, .2, .001
        path = ApproachTrajectory(start, target, speed, acceleration)
        previous = start
        previous_velocity = (0.0,) * 14
        while not path.finished:
            command = path.advance(dt)
            velocity = tuple((b - a) / dt for a, b in zip(previous, command))
            for index, distance in enumerate(target):
                if distance:
                    self.assertAlmostEqual(command[index] / distance, command[0] / target[0], places=12)
                else:
                    self.assertEqual(command[index], 0.0)
                self.assertLessEqual(abs(velocity[index]), speed + 1e-10)
                self.assertLessEqual(abs(velocity[index] - previous_velocity[index]) / dt,
                                     acceleration + 1e-8)
                self.assertEqual(command[index] == distance, command == target or distance == 0)
            previous, previous_velocity = command, velocity
        self.assertEqual(command, target)
        self.assertTrue(path.stopped)
        self.assertEqual(path.advance(dt), target)
        self.assertLessEqual(max(abs(v) for v in previous_velocity) / dt, acceleration + 1e-8)

    def test_pause_brakes_without_velocity_jump_or_overshoot_at_every_path_stage(self):
        dt = .001
        for fraction in (0.0, .01, .2, .5, .8, .99, 1.0):
            with self.subTest(fraction=fraction):
                path = ApproachTrajectory((0.0, 0.0), (.3, -.12), .1, .2)
                previous = path.position
                previous_velocity = (0.0, 0.0)
                for _ in range(math.ceil(path.duration * fraction / dt)):
                    command = path.advance(dt)
                    previous_velocity = tuple((b - a) / dt for a, b in zip(previous, command))
                    previous = command
                velocity_at_pause = path.velocity
                path.brake()
                self.assertEqual(path.velocity, velocity_at_pause)
                for _ in range(1000):
                    command = path.advance(dt)
                    velocity = tuple((b - a) / dt for a, b in zip(previous, command))
                    for v, old_v in zip(velocity, previous_velocity):
                        self.assertLessEqual(abs(v), .1 + 1e-10)
                        self.assertLessEqual(abs(v - old_v) / dt, .2 + 1e-8)
                    self.assertGreaterEqual(command[0], previous[0])
                    self.assertLessEqual(command[0], .3)
                    self.assertAlmostEqual(command[0] / .3, command[1] / -.12, places=12)
                    previous, previous_velocity = command, velocity
                    if path.stopped:
                        break
                self.assertTrue(path.stopped)
                self.assertEqual(path.advance(dt), command)
                self.assertLessEqual(max(abs(v) for v in velocity) / dt, .2 + 1e-8)

    def test_tiny_and_zero_segments_complete_without_nonfinite_commands(self):
        for distance in (0.0, 1e-12, -1e-12):
            path = ApproachTrajectory((0.0, 1.0), (distance, 1.0), .1, .2)
            command = path.advance(.005)
            self.assertEqual(command, (distance, 1.0))
            self.assertTrue(path.finished)
            self.assertTrue(path.stopped)
