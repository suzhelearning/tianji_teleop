"""Behavioral checks for the simulation-only dynamics, without a controller or GUI."""
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

from real_robot.protocol import ARMS_READY, LEFT_HAND_READY, CommandFrame
from control.model_assets import OBJECT_BODY_NAME
from sim.physics import PhysicsSimulation


ROOT = Path(__file__).resolve().parents[2] / "control"


class PhysicsTests(unittest.TestCase):
    def setUp(self):
        self.sim = PhysicsSimulation(
            ROOT / "models/marvin_m6_wuji2.xml",
            ROOT / "config/qp_ik_pico_teleop.yaml",
        )

    def frame(self, positions, flags):
        return CommandFrame(1, 1, 0, flags, tuple(positions[:7]),
                            tuple(positions[7:14]), tuple(positions[14:34]),
                            tuple(positions[34:54]))

    def test_targets_produce_bounded_force_not_teleportation(self):
        sim = self.sim
        before = sim.data.qpos.copy()
        targets = sim.targets.copy()
        targets[0] += 0.2
        targets[16] = 0.5
        sim.set_targets(self.frame(targets, ARMS_READY | LEFT_HAND_READY))
        np.testing.assert_array_equal(sim.data.qpos, before)
        sim.step()
        self.assertAlmostEqual(sim.data.time, sim.model.opt.timestep)
        self.assertGreater(abs(sim.data.actuator_force[0]), 0.1)
        self.assertGreater(abs(sim.data.actuator_force[16]), 0.01)
        self.assertLess(sim.data.qpos[sim.qpos_indices[0]], targets[0])
        for _ in range(999):
            sim.step()
            self.assertTrue(np.all(sim.data.actuator_force >= sim.model.actuator_forcerange[:, 0] - 1e-10))
            self.assertTrue(np.all(sim.data.actuator_force <= sim.model.actuator_forcerange[:, 1] + 1e-10))
        self.assertLess(abs(sim.data.qpos[sim.qpos_indices[16]] - targets[16]), 0.15)

    def test_ready_groups_and_rejection_are_atomic(self):
        sim = self.sim
        before = sim.targets.copy()
        targets = before.copy()
        targets[0] += 0.1
        targets[14] = 0.2
        sim.set_targets(self.frame(targets, LEFT_HAND_READY))
        np.testing.assert_array_equal(sim.targets[:14], before[:14])
        self.assertEqual(sim.targets[14], targets[14])
        before = sim.targets.copy()
        targets[14] = 100.0
        with self.assertRaises(ValueError):
            sim.set_targets(self.frame(targets, ARMS_READY | LEFT_HAND_READY))
        np.testing.assert_array_equal(sim.targets, before)

    def test_unpowered_robot_accelerates_under_gravity(self):
        sim = self.sim
        sim.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
        before = sim.data.qpos.copy()
        for _ in range(20):
            sim.step()
        self.assertGreater(np.linalg.norm(sim.data.qpos - before), 1e-4)
        self.assertGreater(np.linalg.norm(sim.data.qvel), 1e-3)

    def test_external_load_moves_the_servo_held_robot(self):
        sim = self.sim
        for _ in range(500):
            sim.step()
        index = int(sim.model.joint("Joint1_L").dofadr[0])
        before = sim.data.qpos[sim.qpos_indices[0]]
        sim.data.qfrc_applied[index] = 50.0
        for _ in range(500):
            sim.step()
        self.assertGreater(sim.data.qpos[sim.qpos_indices[0]] - before, 0.03)

    def test_contact_with_an_external_obstacle_generates_force(self):
        sim = self.sim
        obstacle = sim.model.geom("target_geom_L").id
        finger_body = sim.model.body("l_index_finger_distal").id
        finger_geom = int(sim.model.body_geomadr[finger_body])
        mocap = int(sim.model.body_mocapid[sim.model.geom_bodyid[obstacle]])
        sim.model.geom_contype[obstacle] = 1
        sim.model.geom_conaffinity[obstacle] = 1
        # This visualization marker was compiled as non-colliding. Broad-phase
        # body masks must also change when enabling its geom at runtime.
        obstacle_body = int(sim.model.geom_bodyid[obstacle])
        sim.model.body_contype[obstacle_body] = 1
        sim.model.body_conaffinity[obstacle_body] = 1
        sim.data.mocap_pos[mocap] = sim.data.geom_xpos[finger_geom]
        sim.step()
        force = np.zeros(6)
        normal_force = 0.0
        for index, contact in enumerate(sim.data.contact):
            if obstacle in contact.geom:
                mujoco.mj_contactForce(sim.model, sim.data, index, force)
                normal_force += abs(force[0])
        self.assertGreater(normal_force, 0.01)

    def test_visual_object_overlapping_robot_does_not_change_dynamics(self):
        with tempfile.TemporaryDirectory() as directory:
            mesh = Path(directory) / "cube.obj"
            mesh.write_text(
                "v -0.1 -0.1 -0.1\nv 0.1 -0.1 -0.1\n"
                "v 0.1 0.1 -0.1\nv -0.1 0.1 -0.1\n"
                "v -0.1 -0.1 0.1\nv 0.1 -0.1 0.1\n"
                "v 0.1 0.1 0.1\nv -0.1 0.1 0.1\n"
                "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\n"
                "f 1 2 6\nf 1 6 5\nf 4 8 7\nf 4 7 3\n"
                "f 1 5 8\nf 1 8 4\nf 2 3 7\nf 2 7 6\n",
                encoding="utf-8",
            )
            decorated = PhysicsSimulation(
                ROOT / "models/marvin_m6_wuji2.xml",
                ROOT / "config/qp_ik_pico_teleop.yaml",
                object_mesh=mesh,
            )
        baseline = self.sim
        targets = baseline.targets.copy()
        targets[0] += 0.1
        targets[16] = 0.3
        frame = self.frame(targets, ARMS_READY | LEFT_HAND_READY)
        baseline.set_targets(frame)
        decorated.set_targets(frame)
        object_geom = decorated.model.geom(OBJECT_BODY_NAME).id
        mocap = decorated.model.body(OBJECT_BODY_NAME).mocapid[0]
        for tick in range(50):
            # A collidable 20 cm cube here would obstruct both finger chains.
            side = "l" if tick < 25 else "r"
            finger = decorated.model.body(f"{side}_index_finger_distal").id
            decorated.data.mocap_pos[mocap] = decorated.data.xpos[finger]
            baseline.step()
            decorated.step()
            self.assertFalse(any(object_geom in contact.geom for contact in decorated.data.contact))
            np.testing.assert_allclose(decorated.data.qpos, baseline.data.qpos, rtol=0, atol=1e-12)
            np.testing.assert_allclose(decorated.data.qvel, baseline.data.qvel, rtol=0, atol=1e-10)
            np.testing.assert_allclose(
                decorated.data.actuator_force, baseline.data.actuator_force, rtol=0, atol=1e-8,
            )

    def test_numerical_failure_cannot_resume_as_success(self):
        sim = self.sim
        sim.data.qvel[0] = np.nan
        with self.assertRaises(RuntimeError):
            sim.step()
        sim.data.qvel[0] = 0.0
        with self.assertRaises(RuntimeError):
            sim.step()


if __name__ == "__main__":
    unittest.main()
