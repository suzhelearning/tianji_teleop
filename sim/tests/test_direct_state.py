from pathlib import Path
import unittest
import types
from unittest.mock import patch
import numpy as np
import mujoco
from real_robot.protocol import ARMS_READY, LEFT_HAND_READY, CommandFrame
from sim.direct_state import DirectStateSimulation
from sim.mapped_recovery import MappedRecovery

ROOT = Path(__file__).resolve().parents[2] / 'control/mapped_palm'


class DirectTests(unittest.TestCase):
    def setUp(self):
        self.sim = DirectStateSimulation(ROOT/'assets/mapped_palm/marvin_m6_wuji2.xml', ROOT/'config/deployment.yaml')

    def frame(self, q, flags=ARMS_READY):
        return CommandFrame(1,1,1,flags,tuple(q[:7]),tuple(q[7:14]),tuple(q[14:34]),tuple(q[34:]))

    def test_exact_joint_display_without_physics_integration(self):
        q = self.sim.targets.copy(); q[0] += .1
        self.sim.set_targets(self.frame(q))
        with patch('mujoco.mj_step', side_effect=AssertionError('must not integrate')):
            self.sim.step()
        np.testing.assert_array_equal(self.sim.data.qpos[self.sim.qpos_indices], q)
        self.assertGreater(abs(self.sim.data.qvel[self.sim._velocity_indices[0]]), 0)
        self.sim.step()
        np.testing.assert_array_equal(self.sim.data.qvel[self.sim._velocity_indices], 0)
        reference=mujoco.MjData(self.sim.model)
        reference.qpos[:]=self.sim.data.qpos
        mujoco.mj_forward(self.sim.model, reference)
        np.testing.assert_allclose(self.sim.data.site_xpos, reference.site_xpos)

    def test_ready_gate_limits_and_no_gravity_drift(self):
        before=self.sim.targets.copy(); q=before.copy(); q[0]+=.2; q[14]=.2
        self.sim.set_targets(self.frame(q,LEFT_HAND_READY)); self.sim.step()
        np.testing.assert_array_equal(self.sim.data.qpos[self.sim.qpos_indices[:14]],before[:14])
        self.assertEqual(self.sim.data.qpos[self.sim.qpos_indices[14]],.2)
        q[0]=100
        with self.assertRaises(ValueError): self.sim.set_targets(self.frame(q))
        self.sim.data.qfrc_applied[:]=10
        for _ in range(100): self.sim.step()
        np.testing.assert_array_equal(self.sim.data.qpos[self.sim.qpos_indices],self.sim.targets)

    def test_home_with_actual_mujoco_state_stops_at_wait_for_ack(self):
        recovery=MappedRecovery(self.sim,types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda:123)))
        q=self.sim.targets.copy(); q[0]+=.2
        self.sim.set_targets(self.frame(q)); self.sim.step(); self.sim.step()
        sent=[]
        recovery.key(ord('H'))
        peak_velocity=0.
        with patch('sim.mapped_recovery.os.write',side_effect=lambda fd,data:sent.append(data) or len(data)):
            for tick in range(3100):
                with patch('sim.mapped_recovery.time.monotonic_ns',return_value=1_000_000_000+tick*1_000_000):
                    recovery.update()
                self.sim.step()
                peak_velocity=max(peak_velocity,float(max(abs(self.sim.data.qvel[recovery.velocity_indices]))))
        np.testing.assert_allclose(self.sim.data.qpos[self.sim.qpos_indices[:14]],recovery.home,atol=1e-12)
        self.assertLessEqual(peak_velocity,.35)
        self.assertEqual(recovery.home_phase,'rearming')
        self.assertEqual(sum(value.startswith(b'R ') for value in sent),1)
        self.assertNotIn(b's',sent)


if __name__=='__main__': unittest.main()
