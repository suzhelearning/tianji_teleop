"""Deterministic dropout policy tests; fake worker has no actuator authority."""
from dataclasses import replace
import unittest
import numpy as np
from pico2_hands.shared_root_core import SharedRootCore
from pico2_hands.tests.test_shared_root import frame, fk, calibrate


class FakeDls:
    def __init__(self):
        self.phase="WAITING"
        self.starts=0
        self.braking=0
        self.destination="HOLD"

    def forward(self,q):
        return fk(q)

    def reset(self,q):
        pass

    def result(self,q,accepted=True):
        return {s:dict(status=self.phase,joints=q[i].copy(),accepted=accepted)
                for i,s in enumerate(("left","right"))}

    def command(self,op,q,now):
        if op==4:
            assert self.phase in ("WAITING","HOLD","HOME_REACHED")
            self.phase="TELEOP";self.starts+=1
        elif op in (5,6):
            self.phase="BRAKING";self.braking=20
            self.destination="HOME_REACHED" if op==5 else "HOLD"
        elif op==7:
            self.braking-=1
            if self.braking<=0:self.phase=self.destination
        return self.result(q)

    def solve(self,q,targets,**kwargs):
        assert self.phase=="TELEOP"
        return self.result(q)


class DropoutTests(unittest.TestCase):
    def setUp(self):
        self.worker=FakeDls()
        self.core=SharedRootCore(np.zeros(54),self.worker,height_m=1.62)
        calibrate(self.core.mapping)
        self.now=2_020_000_000
        self.core.offer(frame(stamp=self.now),self.now)
        self.assertTrue(self.core.action("s",self.now))
        self.now+=5_000_000
        self.core.offer(frame(stamp=self.now),self.now)
        self.core.tick(self.now)

    def dropout(self,gap=60_000_000):
        self.now+=gap
        self.core.tick(self.now)
        self.assertEqual(self.core.phase,"BRAKING")

    def stream(self,n=60,generation=1):
        for _ in range(n):
            self.now+=10_000_000
            self.core.offer(frame(stamp=self.now,generation=generation),self.now)
            self.core.tick(self.now)

    def test_short_loss_stable_frames_resume_only_after_braking(self):
        self.dropout()
        self.stream(12)
        self.assertEqual(self.worker.starts,1)
        self.assertTrue(self.core._auto_resume["qualified"])
        self.stream()
        self.assertEqual(self.worker.starts,2)
        self.assertEqual(self.core.phase,"TELEOP")

    def test_long_loss_requires_manual_s(self):
        self.dropout(1_050_000_000)
        self.stream()
        self.assertEqual(self.worker.starts,1)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertTrue(self.core.action("s",self.now))

    def test_recovery_after_old_window_but_within_one_second_resumes(self):
        self.dropout(600_000_000)
        self.stream()
        self.assertEqual(self.worker.starts,2)
        self.assertEqual(self.core.phase,"TELEOP")

    def test_stability_must_be_established_within_one_second(self):
        self.dropout(950_000_000)
        self.stream()
        self.assertEqual(self.worker.starts,1)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertIsNone(self.core._auto_resume)

    def test_duplicate_frames_cannot_qualify(self):
        self.dropout()
        self.now+=10_000_000
        duplicate=frame(stamp=self.now)
        self.core.offer(duplicate,self.now)
        for _ in range(120):
            self.now+=10_000_000
            self.core.offer(duplicate,self.now)
            self.core.tick(self.now)
        self.assertEqual(self.worker.starts,1)
        self.assertIsNone(self.core._auto_resume)

    def test_gap_after_qualification_cancels_rearm(self):
        self.dropout();self.stream(12)
        self.assertTrue(self.core._auto_resume["qualified"])
        self.now+=60_000_000;self.core.tick(self.now)
        self.stream()
        self.assertEqual(self.worker.starts,1)

    def test_manual_actions_during_brake_cancel_rearm(self):
        for key in ("p"," ","h","q","c"):
            with self.subTest(key=key):
                self.setUp();self.dropout()
                self.core.action(key,self.now)
                self.stream()
                self.assertEqual(self.worker.starts,1)
                self.assertIsNone(self.core._auto_resume)

    def test_manual_pause_never_auto_resumes(self):
        self.assertTrue(self.core.action("p",self.now))
        self.stream()
        self.assertEqual(self.worker.starts,1)

    def test_disconnect_and_new_generation_cancel(self):
        for disconnect in (False,True):
            with self.subTest(disconnect=disconnect):
                self.setUp();self.dropout()
                if disconnect:self.core.disconnected()
                self.stream(generation=1 if disconnect else 2)
                self.assertEqual(self.worker.starts,1)
                self.assertIsNone(self.core._auto_resume)
                self.assertFalse(self.core.mapping.calibration_allows_start)

    def test_disconnect_forces_hold_and_new_calibration_before_start(self):
        self.core.disconnected()
        # Reconnect can race the first simulation tick; it must not be solved
        # under the old TELEOP authorization.
        self.now+=10_000_000
        self.core.offer(frame(stamp=self.now),self.now)
        self.core.tick(self.now)
        self.assertEqual(self.core.phase,"BRAKING")
        self.stream(30)
        self.assertEqual(self.worker.starts,1)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertFalse(self.core.action("s",self.now))
        self.assertTrue(self.core.action("c",self.now))
        self.stream(102)
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.worker.starts,2)

    def test_invalid_tracking_does_not_count_as_stable(self):
        self.dropout()
        for _ in range(120):
            self.now+=10_000_000
            bad=replace(frame(stamp=self.now),head_valid=False)
            self.core.offer(bad,self.now);self.core.tick(self.now)
        self.stream()
        self.assertEqual(self.worker.starts,1)

    def test_mapping_rejection_does_not_auto_resume(self):
        self.dropout()
        for _ in range(120):
            self.now+=10_000_000
            bad=frame(stamp=self.now,move=(5.,0,0))
            self.core.offer(bad,self.now);self.core.tick(self.now)
        self.stream()
        self.assertEqual(self.worker.starts,1)
