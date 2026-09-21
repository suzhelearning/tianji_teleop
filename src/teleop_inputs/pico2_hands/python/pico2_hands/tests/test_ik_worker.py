import unittest
import io
import subprocess
import xml.etree.ElementTree as ET
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.ik_worker import NativeIkWorker, ROOT, REQUEST, RESPONSE_SIZE


@unittest.skipUnless((ROOT / "native/build/pico2-v131/pico2_v131_worker").is_file(),
                     "build isolated V131 worker first")
class NativeIkWorkerTest(unittest.TestCase):
    def test_unexpected_worker_exit_is_not_hidden_by_close(self):
        worker = NativeIkWorker()
        worker.process.terminate()
        worker.process.wait(timeout=2)
        with self.assertRaises(RuntimeError):
            worker.close()
        self.assertTrue(worker.failed)

    def test_wire_path_matches_direct_cpp_600_frame_trace(self):
        model_root = ROOT / "native/models"
        urdf = model_root / "marvin_m6_s_ccs_696_v4.urdf"
        child = subprocess.run([
            str(ROOT / "native/build/pico2-v131/pico2_v131_model_trace"), str(urdf),
            str(model_root / "marvin_m6_qp_pico_fast_kinematics.xml")],
            capture_output=True, text=True, timeout=15, check=True)
        expected = np.loadtxt(io.StringIO(child.stdout))
        joints = {j.find("child").get("link"): j for j in ET.parse(urdf).getroot().findall("joint")}

        def world_rotation(link):
            if link not in joints:
                return Rotation.identity()
            joint = joints[link]
            origin = joint.find("origin")
            angles = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]
            return world_rotation(joint.find("parent").get("link")) * Rotation.from_euler("xyz", angles)

        bases = [world_rotation("Base_L"), world_rotation("Base_R")]
        axes = [np.array([1., .4, .7]), np.array([.3, 1., .6])]
        axes = [a / np.linalg.norm(a) for a in axes]
        seeds = np.array([[1.1, -.6, -1.52, -1.1, 0, 0, 0], [-1.1, -.6, 1.52, -1.1, 0, 0, 0]])
        rows = []
        with NativeIkWorker() as worker:
            homes = worker.reset(seeds)
            for tick in range(600):
                t = tick * .005
                stamp = 1 + (tick // 2) * .01
                offsets = [[.025*np.sin(t), .02*np.sin(.7*t), .015*np.sin(1.3*t)],
                           [-.02*np.sin(t), .015*np.sin(.8*t), .02*np.sin(.9*t)]]
                targets = []
                for index, side in enumerate(("left", "right")):
                    home = homes[side]["achieved_pose"]
                    rotation = (bases[index].inv() * Rotation.from_rotvec(
                        axes[index] * (.08 if index == 0 else .06) * np.sin(t)) *
                        bases[index] * Rotation.from_quat(home[3:]))
                    targets.append(np.r_[home[:3] + bases[index].inv().apply(offsets[index]), rotation.as_quat()])
                result = worker.solve(seeds, targets, source_time=stamp, received_time=stamp, now=1+t)
                rows.append([tick, result["left"]["accepted"], result["right"]["accepted"],
                             *result["left"]["joints"], *result["right"]["joints"]])
        rows = np.array(rows)
        np.testing.assert_array_equal(rows[:, :3], expected[:, :3])
        np.testing.assert_allclose(rows[:, 3:], expected[:, 3:], rtol=0, atol=1e-8)

    def seeds(self):
        return np.deg2rad([[55, -65, -70, -60, 60, 0, 0],
                          [-55, -65, 70, -60, -60, 0, 0]])

    def test_reset_fk_motion_and_repeated_reset(self):
        self.assertEqual(REQUEST.size, 272)
        self.assertEqual(RESPONSE_SIZE, 432)
        seed = self.seeds()
        with NativeIkWorker() as worker:
            fk = worker.forward(seed)
            target = np.array([fk[s]["achieved_pose"] for s in ("left", "right")])
            with self.assertRaises(ValueError):
                worker.solve(seed, target, source_time=1., received_time=1., now=1.)
            worker.reset(seed)
            self.assertEqual(worker.epoch, 1)
            target[:, 0] += .01
            accepted = [0, 0]
            for i in range(100):
                stamp = 1 + i * .005
                result = worker.solve(seed, target, source_time=stamp, received_time=stamp, now=stamp)
                for index, side in enumerate(("left", "right")):
                    accepted[index] += result[side]["accepted"]
            self.assertGreater(min(accepted), 50)
            self.assertGreater(np.linalg.norm(result["left"]["joints"] - seed[0]), 1e-4)
            reset = worker.reset(seed)
            self.assertEqual(worker.epoch, 2)
            np.testing.assert_array_equal(reset["left"]["joints"], seed[0])
            self.assertFalse(reset["left"]["accepted"])
        self.assertEqual(worker.process.returncode, 0)

    def test_invalid_geometry_rejected_before_advancing(self):
        with NativeIkWorker() as worker:
            worker.reset(self.seeds())
            sequence = worker.sequence
            with self.assertRaises(ValueError):
                worker.solve(self.seeds(), np.zeros((2, 7)), source_time=1., received_time=1., now=1.)
            self.assertEqual(worker.sequence, sequence)
            self.assertFalse(worker.failed)

    def test_native_clock_rollback_fails_closed(self):
        with NativeIkWorker() as worker:
            seed = self.seeds()
            fk = worker.reset(seed)
            targets = [fk[s]["achieved_pose"] for s in ("left", "right")]
            worker.solve(seed, targets, source_time=2., received_time=2., now=2.)
            with self.assertRaises(RuntimeError):
                worker.solve(seed, targets, source_time=1., received_time=3., now=3.)
            self.assertTrue(worker.failed)
            with self.assertRaises(RuntimeError):
                worker.reset(seed)

    def test_timeout_latches_client(self):
        with NativeIkWorker() as worker:
            with patch.object(worker, "_read", side_effect=TimeoutError("injected")):
                with self.assertRaises(TimeoutError):
                    worker.reset(self.seeds())
            self.assertTrue(worker.failed)
