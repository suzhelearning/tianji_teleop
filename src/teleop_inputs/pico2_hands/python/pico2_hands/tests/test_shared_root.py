"""Offline synthetic input only; never contacts PICO/ADB or hardware."""
from dataclasses import replace
import subprocess
import sys
import unittest
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from pico2_hands.reference.pico import parse_pico_packet
from pico2_hands.tests.test_pico_hand_tracking import _packet
from pico2_hands.shared_root_mapping import SharedRootMapping
from pico2_hands.resources import display_model_path
from tianji_runtime.resources import controller_profile, native_executable, ResourceNotFound
from pico2_hands.shared_root_core import SharedRootCore
from pico2_hands.dls_worker import DlsWorker


def dls_available():
    try:
        native_executable("pico2_dls_worker")
    except ResourceNotFound:
        return False
    return True


class CliTests(unittest.TestCase):
    def test_cli_rejects_missing_height_and_legacy_height(self):
        for args in (["--mapping-mode","shared-root"],["--height-m","1.62"],
                     ["--mapping-mode","shared-root","--height-m","nan"]):
            result=subprocess.run([sys.executable,"-m","pico2_hands.run_sim",*args],capture_output=True,timeout=10)
            self.assertEqual(result.returncode,2)


def frame(height=1.62, stamp=1_000_000_000, yaw=0., generation=1, move=None):
    raw=parse_pico_packet(_packet(),receiver_instance_id="synthetic",connection_generation=generation,
        receiver_frame_sequence=stamp,received_timestamp_ns=stamp)
    r=Rotation.from_euler("z",yaw)
    head=r.apply([0.,0,height*.93])
    hands={}
    for i,side in enumerate(("left","right")):
        wrist=r.apply([height*(.155882+.152941), (1 if i==0 else -1)*height*.1828/2, height*.80])
        if move is not None:
            wrist+=r.apply(np.array(move)*height)
        hand=raw.hands[side]
        joints=list(hand.joints)
        for j in range(26):
            joints[j]=replace(joints[j],pose=np.r_[wrist+r.apply([.05,j*.0001,0]),r.as_quat()])
        joints[12]=replace(joints[12],pose=np.r_[wrist+r.apply([height*.05,0,0]),r.as_quat()])
        hands[side]=replace(hand,wrist_pose=np.r_[wrist,r.as_quat()],joints=tuple(joints))
    return replace(raw,source_timestamp_ms=stamp//1_000_000,head_pose=np.r_[head,r.as_quat()],hands=hands)


def fk(q):
    return {s:dict(achieved_pose=np.array([.7,sign*.2115,1.121,0,0,0,1.])) for s,sign in (("left",1),("right",-1))}


def calibrate(mapping,height=1.62,yaw=0., start=1_000_000_000):
    assert mapping.request_calibration(start,idle=True)
    for n in range(51):
        now=start+n*20_000_000
        mapping.offer_frame(frame(height,now,yaw),now)
        mapping.tick(now)
    assert mapping.state=="calibrated",mapping.status()
    return now


class MappingTests(unittest.TestCase):
    def test_calibration_completes_at_one_second_not_before(self):
        m=SharedRootMapping(1.62,fk);start=1_000_000_000
        self.assertTrue(m.request_calibration(start,idle=True))
        for n in range(50):
            now=start+n*20_000_000
            m.offer_frame(frame(stamp=now),now)
            self.assertFalse(m.tick(now))
            self.assertEqual(m.state,"collecting")
        now=start+1_000_000_000
        m.offer_frame(frame(stamp=now),now)
        self.assertTrue(m.tick(now))
        self.assertTrue(m.calibration_allows_start)

    def test_one_second_still_rejects_too_few_samples(self):
        m=SharedRootMapping(1.62,fk);start=1_000_000_000
        m.request_calibration(start,idle=True)
        for n in range(11):
            now=start+n*100_000_000
            m.offer_frame(frame(stamp=now),now);m.tick(now)
        self.assertEqual(m.state,"failed")
        self.assertFalse(m.calibration_allows_start)

    def test_required_c_height_and_geometry(self):
        for bad in (None,True,float("nan"),float("inf"),162,.9,2.5):
            with self.assertRaises(ValueError): SharedRootMapping(bad,fk)
        m=SharedRootMapping(1.62,fk)
        self.assertFalse(m.calibration_allows_start)
        self.assertAlmostEqual(m.width,.296136)
        self.assertAlmostEqual(m.palm_distance,.05999994)
        with self.assertRaises(ValueError): m.targets(frame())
        calibrate(m)
        self.assertTrue(m.calibration_allows_start)

    def test_scaled_people_and_calibrated_yaw_map_identically(self):
        expected=None
        for h in (1.45,1.62,1.85,2.0):
            for yaw in (0.,.7,-1.2):
                m=SharedRootMapping(h,fk);calibrate(m,h,yaw)
                target=m.targets(frame(h,4_000_000_000,yaw,move=(-.05,.02,-.1)))
                if expected is None: expected=target
                np.testing.assert_allclose(target,expected,atol=1e-12)

    def test_single_offset_head_rotation_independence_and_hand_orientation(self):
        m=SharedRootMapping(1.62,fk);calibrate(m)
        f=frame(); original=m.targets(f)
        changed=replace(f,head_pose=np.r_[f.head_pose[:3],Rotation.from_euler("xyz",[.3,.2,.8]).as_quat()])
        np.testing.assert_allclose(m.targets(changed),original,atol=1e-12)
        hands=dict(f.hands); hand=hands["left"]; joints=list(hand.joints)
        joints[0]=replace(joints[0],pose=np.array([100,100,100,0,0,0,1.]))
        hands["left"]=replace(hand,joints=tuple(joints))
        np.testing.assert_allclose(m.targets(replace(f,hands=hands)),original,atol=1e-12)
        np.testing.assert_allclose(original[:,0],m.origin[0]+m.scale[0]*m.reach,atol=1e-12)
        joints[12]=replace(joints[12],valid=False)
        hands["left"]=replace(hand,joints=tuple(joints))
        with self.assertRaises(ValueError): m.targets(replace(f,hands=hands))

    def test_repeat_c_no_accumulation_and_failure_blocks_start(self):
        m=SharedRootMapping(1.62,fk);calibrate(m)
        old=m.targets(frame())
        calibrate(m,start=4_000_000_000)
        np.testing.assert_allclose(m.targets(frame()),old,atol=1e-12)
        self.assertFalse(m.request_calibration(7_000_000_000,idle=False))
        m.request_calibration(7_000_000_000,idle=True)
        m.tick(7_300_000_000)
        self.assertEqual(m.state,"failed")
        self.assertFalse(m.calibration_allows_start)

    def test_moving_and_wrong_spread_calibration_rejected(self):
        for variant in ("motion","spread","heading"):
            m=SharedRootMapping(1.62,fk);start=1_000_000_000;m.request_calibration(start,idle=True)
            for i in range(51):
                now=start+i*20_000_000;f=frame(stamp=now)
                if variant=="motion": f=frame(stamp=now,move=(i*.001,0,0))
                elif variant=="heading": f=frame(stamp=now,yaw=i*.005)
                else:
                    hands=dict(f.hands);hands["right"]=hands["left"];f=replace(f,hands=hands)
                m.offer_frame(f,now);m.tick(now)
            self.assertEqual(m.state,"failed",variant)


@unittest.skipUnless(dls_available(), "build main DLS worker")
class NativeTests(unittest.TestCase):
    def setUp(self):
        config=yaml.safe_load(controller_profile("qp_ik_pico_shared_root_dls.yaml").read_text())["controller"]
        self.home=np.r_[config["initial_left_q_rad"],config["initial_right_q_rad"],np.zeros(40)]

    def test_dls_start_brake_home_and_display(self):
        from pico2_hands.display_contract import validate_dls_display
        from simulation.direct_state import DirectStateSimulation
        sim=DirectStateSimulation(display_model_path("marvin_m6_wuji2_shared_root_ceres.xml"),controller_profile("qp_ik_pico_shared_root_dls.yaml"))
        with DlsWorker() as ik:
            validate_dls_display(sim,ik)
            q=self.home[:14].reshape(2,7)
            ik.reset(q)
            target=np.array([ik.forward(q)[s]["achieved_pose"] for s in ("left","right")]);target[:,0]+=.04
            self.assertTrue(ik.command(4,q,1.)["left"]["accepted"])
            previous=q.copy()
            for i in range(100):
                now=1.005+i*.005
                r=ik.solve(q,target,source_time=now,received_time=now,now=now)
                self.assertTrue(r["left"]["accepted"])
                q=np.array([r[s]["joints"] for s in ("left","right")])
                self.assertLess(np.max(abs(q-previous))/.005,1.401)
                previous=q.copy()
            self.assertGreater(np.max(abs(q-self.home[:14].reshape(2,7))),.0001)
            with self.assertRaises(RuntimeError):
                # Separate worker below exercises recovery; reset while moving
                # must fail closed and latch this worker, not zero velocity.
                ik.reset(q)

    def test_core_calibration_reconnect_stale_and_home(self):
        with DlsWorker() as ik:
            core=SharedRootCore(self.home,ik,height_m=1.62)
            now=1_000_000_000
            core.offer(frame(stamp=now),now)
            self.assertFalse(core.action("s",now))
            self.assertTrue(core.action("c",now))
            for i in range(51):
                now+=20_000_000;core.offer(frame(stamp=now),now);core.tick(now)
            self.assertEqual(core.mapping.state,"calibrated")
            now+=20_000_000;core.offer(frame(stamp=now),now)
            self.assertTrue(core.action("s",now))
            self.assertFalse(core.action("c",now))
            core.q[14:]=.3  # Fingers must hold, not enter the old Home interpolation.
            for _ in range(30):
                now+=5_000_000;core.offer(frame(stamp=now),now);core.tick(now)
            now+=100_000_000;core.tick(now)
            self.assertEqual(core.phase,"BRAKING")
            self.assertFalse(core.action("s",now))
            self.assertTrue(core.action("q",now))
            for _ in range(5000):
                now+=5_000_000;core.tick(now)
                if core.done: break
            self.assertTrue(core.done)
            np.testing.assert_allclose(core.q[:14],self.home[:14],atol=1e-6)
            np.testing.assert_array_equal(core.q[14:],np.full(40,.3))
            now+=5_000_000;core.offer(frame(stamp=now,generation=2),now)
            self.assertFalse(core.mapping.calibration_allows_start)


    def test_reconnect_source_clock_reset_requires_stationary_calibrated_start(self):
        with DlsWorker() as ik:
            core=SharedRootCore(self.home,ik,height_m=1.62)
            calibrate(core.mapping)
            now=2_020_000_000
            core.offer(replace(frame(stamp=now),source_timestamp_ms=100_000),now)
            self.assertTrue(core.action("s",now))
            now+=5_000_000;core.tick(now)
            epoch=ik.epoch
            now+=5_000_000
            core.offer(replace(frame(stamp=now,generation=2),source_timestamp_ms=1),now)
            self.assertEqual(ik.epoch,epoch)
            self.assertFalse(core.action("s",now))
            for _ in range(5000):
                now+=5_000_000;core.tick(now)
                if core.phase=="HOME_REACHED":break
            self.assertEqual(core.phase,"HOME_REACHED")
            self.assertEqual(ik.epoch,epoch)
            self.assertTrue(core.action("c",now))
            for i in range(51):
                now+=20_000_000
                core.offer(replace(frame(stamp=now,generation=2),source_timestamp_ms=20+i*20),now)
                core.tick(now)
            now+=5_000_000
            core.offer(replace(frame(stamp=now,generation=2),source_timestamp_ms=1100),now)
            self.assertTrue(core.action("s",now))
            self.assertEqual(ik.epoch,epoch+1)
            now+=5_000_000;core.tick(now)
            self.assertEqual(core.phase,"TELEOP")
            self.assertFalse(core.offer(replace(frame(stamp=now+1,generation=2),source_timestamp_ms=1000),now+1))

    def test_pause_cancels_home_during_braking_and_homing(self):
        for phase,key in (("BRAKING","p"),("HOMING"," ")):
            with self.subTest(phase=phase), DlsWorker() as ik:
                core=SharedRootCore(self.home,ik,height_m=1.62)
                calibrate(core.mapping);now=2_020_000_000
                core.offer(frame(stamp=now),now);self.assertTrue(core.action("s",now))
                for _ in range(80):
                    now+=5_000_000;core.offer(frame(stamp=now),now);core.tick(now)
                self.assertTrue(core.action("h",now))
                for _ in range(4000):
                    if core.phase==phase:break
                    now+=5_000_000;core.tick(now)
                self.assertEqual(core.phase,phase)
                # Also cancel a deferred Home requested while already braking.
                self.assertTrue(core.action("h",now))
                self.assertTrue(core.action(key,now))
                self.assertFalse(core.pending_home)
                for _ in range(4000):
                    now+=5_000_000;core.tick(now)
                    if core.phase=="HOLD":break
                self.assertEqual(core.phase,"HOLD")
                held=core.q.copy()
                for _ in range(100):
                    now+=5_000_000;core.offer(frame(stamp=now),now);core.tick(now)
                np.testing.assert_array_equal(core.q,held)
                self.assertTrue(core.action("q",now))
                self.assertFalse(core.action(key,now))
                for _ in range(5000):
                    now+=5_000_000;core.tick(now)
                    if core.done:break
                self.assertTrue(core.done)

    def test_native_short_dropout_brakes_then_soft_restarts(self):
        with DlsWorker() as ik:
            core=SharedRootCore(self.home,ik,height_m=1.62)
            calibrate(core.mapping)
            now=2_020_000_000
            core.offer(frame(stamp=now),now)
            self.assertTrue(core.action("s",now))
            for _ in range(30):
                now+=5_000_000;core.offer(frame(stamp=now),now);core.tick(now)
            now+=60_000_000;core.tick(now)
            self.assertEqual(core.phase,"BRAKING")
            resumed=False
            for _ in range(600):
                now+=5_000_000
                core.offer(frame(stamp=now),now)
                before=core.q[:14].copy()
                core.tick(now)
                self.assertLess(np.max(abs(core.q[:14]-before)),4*.005+1e-6)
                if core.phase=="TELEOP":
                    resumed=True
                    break
            self.assertTrue(resumed,"same-stream stable recovery must resume after rest")
            for _ in range(30):
                now+=5_000_000;core.offer(frame(stamp=now),now)
                before=core.q[:14].copy();core.tick(now)
                self.assertEqual(core.phase,"TELEOP")
                self.assertLess(np.max(abs(core.q[:14]-before)),1.4*.005+1e-6)
