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
        assert self.phase in ("WAITING","HOLD","HOME_REACHED"), "reset while moving"

    def result(self,q,accepted=True):
        return {s:dict(status=self.phase,joints=q[i].copy(),accepted=accepted)
                for i,s in enumerate(("left","right"))}

    def command(self,op,q,now):
        if op in (4,8):
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


class ContinuousFollowTests(unittest.TestCase):
    def setUp(self):
        self.worker=FakeDls()
        self.core=SharedRootCore(np.zeros(54),self.worker,height_m=1.62,continuous_follow=True)
        self.now=1_000_000_000
        self.generation=1
        self.core.offer(frame(stamp=self.now),self.now)

    def stream(self,n=60):
        for _ in range(n):
            self.now+=10_000_000
            self.core.offer(frame(stamp=self.now,generation=self.generation),self.now)
            self.core.tick(self.now)

    def follow(self):
        self.assertTrue(self.core.action("r",self.now))
        self.stream(111)
        self.assertEqual(self.core.state,"idle")
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.core.phase,"TELEOP")

    def test_calibration_waits_for_explicit_s_and_new_fresh_frame(self):
        self.assertFalse(self.core.action("s",self.now))
        self.assertTrue(self.core.action("r",self.now))
        self.stream(100)
        self.assertTrue(self.core.mapping.calibration_allows_start)
        self.assertFalse(self.core.action("s",self.now))
        ready_q=self.core.q.copy()
        self.stream(50)
        self.assertEqual(self.core.state,"idle")
        self.assertEqual(self.worker.starts,0)
        np.testing.assert_array_equal(self.core.q,ready_q)
        self.assertEqual(self.core.spd_output(self.now)[0],1)
        self.now+=60_000_000
        self.assertFalse(self.core.action("s",self.now))
        self.stream(1)
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,1)

    def test_long_dropout_restores_same_session_without_r_or_s(self):
        self.follow()
        self.now+=5_000_000_000
        self.core.tick(self.now)
        self.assertEqual(self.core.phase,"BRAKING")
        self.stream(12)
        self.assertEqual(self.worker.starts,1)
        self.assertEqual(self.core.phase,"BRAKING")
        self.stream(30)
        self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,2)

    def test_gap_after_stable_recovery_restarts_qualification_not_authorization(self):
        self.follow()
        self.now+=60_000_000
        self.core.tick(self.now)
        self.stream(12)
        self.now+=5_000_000_000
        self.core.tick(self.now)
        self.assertEqual(self.worker.starts,1)
        self.stream(30)
        self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,2)

    def test_cached_or_duplicate_frames_cannot_rearm(self):
        self.follow()
        duplicate=self.core.frame
        self.now+=5_000_000_000
        self.core.tick(self.now)
        for _ in range(60):
            self.now+=10_000_000
            self.assertFalse(self.core.offer(duplicate,self.now))
            self.core.tick(self.now)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertEqual(self.worker.starts,1)
        self.stream(11)
        self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,2)

    def test_disconnect_or_identity_change_requires_new_r_then_s_and_never_homes(self):
        for change in ("disconnect","generation","receiver"):
            with self.subTest(change=change):
                self.setUp()
                self.follow()
                if change=="disconnect":
                    self.core.disconnected()
                elif change=="generation":
                    self.generation=2
                self.now+=10_000_000
                incoming=frame(stamp=self.now,generation=self.generation)
                if change=="receiver":
                    incoming=replace(incoming,receiver_instance_id="replacement")
                self.core.offer(incoming,self.now)
                self.core.tick(self.now)
                self.assertEqual(self.core.phase,"BRAKING")
                self.assertEqual(self.core.spd_output(self.now)[0],0)
                self.stream(150)
                self.assertEqual(self.core.phase,"HOLD")
                self.assertEqual(self.worker.starts,1)
                self.assertFalse(self.core.mapping.calibration_allows_start)
                self.assertFalse(self.core.action("s",self.now))
                self.follow()
                self.assertEqual(self.worker.starts,2)

    def test_r_while_following_brakes_before_sampling_even_when_repeated(self):
        self.follow()
        self.assertTrue(self.core.action("r",self.now))
        self.assertEqual(self.core.phase,"BRAKING")
        self.assertFalse(self.core.mapping.calibration_allows_start)
        self.assertFalse(self.core.action("s",self.now))
        self.assertEqual(self.core.spd_output(self.now)[0],0)
        self.stream(10)
        self.assertTrue(self.core.action("r",self.now))
        self.stream(9)
        self.assertEqual(self.core.phase,"BRAKING")
        self.assertNotEqual(self.core.mapping.state,"collecting")
        self.assertEqual(self.core.mapping.samples,[])
        self.assertEqual(self.worker.starts,1)
        # R must not restart the active brake; its original 20 ticks reach HOLD.
        self.stream(1)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertEqual(self.core.mapping.state,"collecting")
        self.assertEqual(self.core.mapping.samples,[])
        self.stream(99)
        self.assertFalse(self.core.mapping.calibration_allows_start)
        self.assertEqual(self.worker.starts,1)
        self.stream(12)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertEqual(self.worker.starts,1)
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,2)

    def test_repeated_r_during_collection_restarts_sampling_window(self):
        self.assertTrue(self.core.action("r",self.now))
        self.stream(80)
        self.assertTrue(self.core.action("r",self.now))
        self.stream(99)
        self.assertFalse(self.core.mapping.calibration_allows_start)
        self.assertEqual(self.worker.starts,0)
        self.stream(12)
        self.assertEqual(self.core.state,"idle")
        self.assertEqual(self.worker.starts,0)
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.core.phase,"TELEOP")

    def test_failed_recalibration_never_resumes_old_mapping(self):
        self.follow()
        self.assertTrue(self.core.action("r",self.now))
        self.stream(20)
        self.now+=10_000_000
        self.core.offer(replace(frame(stamp=self.now),head_valid=False),self.now)
        self.core.tick(self.now)
        self.stream(150)
        self.assertFalse(self.core.mapping.calibration_allows_start)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertEqual(self.worker.starts,1)
        self.assertEqual(self.core.spd_output(self.now)[0],0)
        self.follow()
        self.assertEqual(self.worker.starts,2)

    def test_manual_pause_requires_s_without_recalibration(self):
        self.follow()
        self.assertTrue(self.core.action("p",self.now))
        self.stream(150)
        self.assertEqual(self.core.phase,"HOLD")
        self.assertTrue(self.core.mapping.calibration_allows_start)
        self.assertEqual(self.worker.starts,1)
        self.assertTrue(self.core.action("s",self.now))
        self.assertEqual(self.core.phase,"TELEOP")

    def test_home_during_pending_recalibration_cancels_sampling(self):
        self.follow()
        self.assertTrue(self.core.action("r",self.now))
        self.assertTrue(self.core.action("h",self.now))
        self.stream(150)
        self.assertEqual(self.core.phase,"HOME_REACHED")
        self.assertFalse(self.core.mapping.calibration_allows_start)
        self.assertFalse(self.core.action("s",self.now))

    def test_c_and_redundant_s_do_not_interrupt_follow(self):
        self.follow()
        for key in ("c","s"):
            self.assertFalse(self.core.action(key,self.now))
            self.stream(1)
            self.assertEqual(self.core.phase,"TELEOP")
        self.assertEqual(self.worker.starts,1)
