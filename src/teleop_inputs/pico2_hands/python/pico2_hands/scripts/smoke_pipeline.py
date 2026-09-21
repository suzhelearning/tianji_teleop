"""Offline synthetic raw PICO -> mapping/V131 + Hand2 smoke. No publication.

This intentionally does not instantiate a receiver, viewer, SDK or executor.
The fixture wrist poses are derived from Home FK so the test remains reachable.
"""
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pico2_hands.ik_worker import NativeIkWorker
from pico2_hands.hand_worker import NativeHandWorker
from pico2_hands.mapping import OptionalHeightMapping
from pico2_hands.reference.official_pico import (
    pico_official_hand_observations, pico_official_hand2_retarget_input)
from pico2_hands.reference.runtime import pico_frame_observations
from pico2_hands.reference.pico import PICO_TO_MEDIAPIPE, parse_pico_packet
from pico2_hands.tests.test_pico_hand_tracking import _packet


def inverse_fixed_mapping(mapper, side, tcp):
    base_head = Rotation.from_matrix(mapper._rotations[side])
    tracked = mapper._tracked_to_tcp[side]
    wrist_rotation = (base_head.inv() * Rotation.from_quat(tcp[3:]) *
                      mapper._local_corrections[side].inv() * Rotation.from_quat(tracked[3:]).inv())
    wrist_position = (base_head.inv().apply(tcp[:3] - mapper._origins[side] -
                                          mapper._position_offsets[side]) -
                      wrist_rotation.apply(tracked[:3]))
    return np.r_[wrist_position, wrist_rotation.as_quat()]


def main():
    seed = np.deg2rad([[55, -65, -70, -60, 60, 0, 0], [-55, -65, 70, -60, -60, 0, 0]])
    mapping = OptionalHeightMapping()
    frame = parse_pico_packet(_packet(), receiver_instance_id="offline",
                              connection_generation=1, receiver_frame_sequence=0,
                              received_timestamp_ns=1_000_000_000)
    accepted = {s: 0 for s in ("left", "right")}
    with NativeIkWorker() as ik, NativeHandWorker("left") as left, NativeHandWorker("right") as right:
        reference = ik.reset(seed)
        for tick in range(60):
            stamp_ns = 1_000_000_000 + tick * 5_000_000
            hands = {}
            for side, hand in frame.hands.items():
                tcp = reference[side]["achieved_pose"].copy()
                tcp[0] += .003 * np.sin(tick * .03)
                wrist = inverse_fixed_mapping(mapping.mapper, side, tcp)
                joints = list(hand.joints)
                for index, pico_index in enumerate(PICO_TO_MEDIAPIPE):
                    if index == 0:
                        position = wrist[:3]
                    else:
                        finger, joint = divmod(index - 1, 4)
                        position = wrist[:3] + np.array([(finger - 2)*.02, (joint + 1)*.025, .002*joint])
                    joints[pico_index] = replace(joints[pico_index], pose=np.r_[position, wrist[3:]])
                hands[side] = replace(hand, wrist_pose=wrist, joints=tuple(joints))
            current = replace(frame, head_pose=np.array([0., 0., 0., 0., 0., 0., 1.]),
                              hands=hands, source_timestamp_ms=stamp_ns // 1_000_000,
                              received_timestamp_ns=stamp_ns, receiver_frame_sequence=tick)
            observations = pico_frame_observations(current)
            targets = np.array([mapping.map(observations[s][1]).pose for s in ("left", "right")])
            result = ik.solve(seed, targets, source_time=stamp_ns / 1e9,
                              received_time=stamp_ns / 1e9, now=stamp_ns / 1e9)
            official = pico_official_hand_observations(current)
            for side, worker in (("left", left), ("right", right)):
                if not official[side].valid:
                    raise AssertionError("invalid synthetic hand geometry")
                q = worker.retarget(pico_official_hand2_retarget_input(official[side].keypoints_m),
                                    tick + 1, stamp_ns)
                if not np.isfinite(q).all():
                    raise AssertionError("nonfinite hand output")
                accepted[side] += result[side]["accepted"]
        if min(accepted.values()) < 30:
            raise AssertionError(f"insufficient accepted IK cycles: {accepted}")
    print(json.dumps(dict(scope="synthetic_pico2_compute_pipeline", frames=60,
                         accepted=accepted, c_requested=False, robot_commands_enabled=False,
                         hardware_acceptance_complete=False)))


if __name__ == "__main__":
    main()
