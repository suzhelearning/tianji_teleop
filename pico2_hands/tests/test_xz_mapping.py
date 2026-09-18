from dataclasses import replace
import numpy as np

from pico2_hands.xz_mapping import OptionalXZMapping
from pico2_hands.tests import test_optional_calibration as fixtures
from pico2_hands.ik_worker import NativeIkWorker


def test_native_reference_xz_alignment_retains_y_rotation_and_motion():
    helper = fixtures.OptionalCalibrationTest()
    with NativeIkWorker(timeout_s=1) as ik:
        mapping = OptionalXZMapping(ik.forward)
        samples = {s: helper.sample(s, 1_000_000_000, x=.4 if s == "left" else .5)
                   for s in ("left", "right")}
        before = {s: mapping.map(v).pose.copy() for s,v in samples.items()}
        assert mapping.request_calibration(1_000_000_000, idle=True)
        for i in range(101):
            now = 1_000_000_000+i*20_000_000
            for sample in samples.values():
                mapping.add(replace(sample, received_timestamp_ns=now), now)
            mapping.tick(now)
        assert mapping.calibration.state == "calibrated"
        for s, sample in samples.items():
            rotation = mapping.mapper._rotations[s]
            after = mapping.map(sample).pose
            projected = rotation.T @ after[:3]
            np.testing.assert_allclose(projected[[0,2]], mapping.reference[s][[0,2]], atol=1e-12)
            np.testing.assert_allclose(projected[1], (rotation.T @ before[s][:3])[1], atol=1e-12)
            np.testing.assert_allclose(after[3:], before[s][3:])
            moved = sample.pose.copy(); moved[0] -= .1
            delta = rotation.T @ (mapping.map(replace(sample, pose=moved)).pose[:3] - after[:3])
            np.testing.assert_allclose(delta, [-.1,0,0], atol=1e-12)
        old = {s: mapping.map(v).pose.copy() for s,v in samples.items()}
        # Repeated calibration must not accumulate offsets.
        mapping.request_calibration(4_000_000_000, idle=True)
        for i in range(101):
            now = 4_000_000_000+i*20_000_000
            for sample in samples.values():
                mapping.add(replace(sample, received_timestamp_ns=now), now)
            mapping.tick(now)
        for s,v in samples.items():
            np.testing.assert_allclose(mapping.map(v).pose, old[s])
        mapping.request_calibration(7_000_000_000, idle=True)
        mapping.tick(7_300_000_000)
        assert mapping.calibration.state == "failed"
        for s,v in samples.items():
            np.testing.assert_allclose(mapping.map(v).pose, old[s])
