import types
import unittest
from unittest.mock import patch
import numpy as np
from ..mapped_recovery import MappedRecovery


class RecoveryTests(unittest.TestCase):
    def test_native_status_partial_lines_visible_without_blocking(self):
        self.control.controller.stdout=types.SimpleNamespace(fileno=lambda:124)
        with patch('simulation.mapped_recovery.os.read',side_effect=[b'MAPPED: S rej',BlockingIOError()]):
            self.update(1_000_000_000)
        self.assertNotIn('rejected',self.control.status)
        with patch('simulation.mapped_recovery.os.read',side_effect=[b'ected: stale PICO input\n',BlockingIOError()]):
            self.update(1_010_000_000)
        self.assertEqual(self.control.status,'MAPPED: S rejected: stale PICO input')

    def setUp(self):
        names=[f'Joint{i}_{s}' for s in ('L','R') for i in range(1,8)]
        model=types.SimpleNamespace(jnt_dofadr=np.arange(14),joint=lambda name:types.SimpleNamespace(id=names.index(name)))
        self.sim=types.SimpleNamespace(model=model,qpos_indices=np.arange(14),
            targets=np.arange(14)/100.,data=types.SimpleNamespace(qpos=np.arange(14)/100.,qvel=np.zeros(14)))
        self.sim.set_simulation_arm_targets=lambda values: self.sim.targets.__setitem__(slice(None),values)
        self.control=MappedRecovery(self.sim,types.SimpleNamespace(stdin=types.SimpleNamespace(fileno=lambda:123)))
        self.sent=[]

    def update(self, now):
        with patch('simulation.mapped_recovery.time.monotonic_ns',return_value=now),patch('simulation.mapped_recovery.os.write',side_effect=lambda fd,data:self.sent.append(data) or len(data)):
            self.control.update()

    def test_r_requires_continuous_settling_and_carries_measured_state(self):
        self.control.key(ord('R')); self.update(1_000_000_000)
        self.assertEqual(self.sent,[])
        self.control.key(ord('R')); self.update(1_310_000_000)
        values=self.sent[-1].decode().split()
        self.assertEqual(values[:2],['R','1310000000'])
        self.assertEqual(list(map(float,values[2:16])),list(self.sim.data.qpos))
        self.assertEqual(list(map(float,values[16:])),[0.]*14)

    def test_motion_or_nan_cancels_settled_window(self):
        self.update(1_000_000_000)
        self.sim.data.qvel[0]=.04; self.update(1_400_000_000)
        self.sim.data.qvel[0]=0; self.control.key(ord('R')); self.update(1_500_000_000)
        self.assertFalse(self.sent)
        self.sim.data.qpos[0]=float('nan'); self.control.key(ord('R')); self.update(2_000_000_000)
        self.assertFalse(self.sent)

    def test_manual_p_s_still_forwarded(self):
        for key in 'PS': self.control.key(ord(key))
        self.update(1_000_000_000)
        self.assertEqual(self.sent,[b'p',b's'])

    def test_home_rejects_s_waits_measured_rest_and_native_ack(self):
        self.sim.targets[:]+=.2
        self.sim.data.qpos[:]=self.sim.targets
        self.control.key(ord('H')); self.control.key(ord('S'))
        self.update(1_000_000_000)
        self.assertEqual(self.sent,[b'p'])
        self.assertFalse(self.control.accept_frame(types.SimpleNamespace(timestamp_ns=1)))
        self.update(1_100_000_000); self.update(1_500_000_000)
        self.assertEqual(self.control.home_phase,'homing')
        self.update(2_500_000_000)
        np.testing.assert_allclose(self.sim.targets,self.control.home+.1)
        self.update(3_500_000_000)
        self.assertEqual(self.sent,[b'p']) # command at Home != measured at Home
        self.sim.data.qpos[:]=self.control.home
        self.update(3_600_000_000); self.update(3_910_000_000)
        self.assertTrue(self.sent[-1].startswith(b'R '))
        self.assertEqual(self.control.home_phase,'rearming')
        self.control.native_status('MAPPED: R accepted at measured rest; wait for new input then S')
        self.assertEqual(self.control.home_phase,'ready')
        self.assertNotIn(b's',self.sent)
        self.control.key(ord('S')); self.update(4_000_000_000)
        self.assertEqual(self.sent[-1],b's')
        self.assertFalse(self.control.accept_frame(types.SimpleNamespace(timestamp_ns=10**20)))
        self.control.native_status('MAPPED: simulation input started')
        self.assertTrue(self.control.accept_frame(types.SimpleNamespace(timestamp_ns=10**20)))

    def test_home_abort_and_failed_rearm_do_not_resume(self):
        self.control.key(ord('H')); self.update(1_000_000_000)
        self.control.key(ord('P')); self.control.key(ord('S')); self.update(1_100_000_000)
        self.assertEqual(self.control.home_phase,'home_hold')
        self.assertNotIn(b's',self.sent)
        self.control.home_phase='rearming'
        self.control.native_status('MAPPED: R rejected: stale input')
        self.assertEqual(self.control.home_phase,'home_hold')


if __name__=='__main__': unittest.main()
