from tianji_runtime.resources import controller_profile, package_share
import unittest
from unittest.mock import patch
import numpy as np
import mujoco
from tianji_controller.protocol import ARMS_READY, LEFT_HAND_READY, CommandFrame
from ..direct_state import DirectStateSimulation



class DirectTests(unittest.TestCase):
    def setUp(self):
        DESCRIPTION = package_share("tianji_description")
        self.sim = DirectStateSimulation(DESCRIPTION/'models/marvin_m6_wuji2_shared_root_ceres.xml',
                          controller_profile("qp_ik_pico_shared_root_dls.yaml"))

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



if __name__=='__main__': unittest.main()
