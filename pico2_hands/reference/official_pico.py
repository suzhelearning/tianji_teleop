"""PICO FLU geometry boundary for the pinned official Manus Hand2 YAMLs.

PICO_2 already sends right-handed FLU meters. The pinned Manus callback
reflects Y; its YAML then reflects Z before estimating the wrist frame. Apply
that same chirality boundary here: the combined Y/Z reflections are a proper
rotation, removed by official wrist-frame estimation, not a mirrored hand.

The PICO hand model is slightly shorter than the pinned Wuji Hand2 model in
the two recent device recordings. The official worker therefore receives a
PICO-only radial geometry scale around the wrist. Raw observations remain
unscaled so overlays and recordings describe the device data exactly.

This does not transform arm targets, subtract head/wrist position, retarget,
pack Manus callbacks, or authorize execution. Legacy PICO conversion and the
Manus route remain unchanged. Physical PICO retarget calibration still
requires device testing.
"""
import numpy as np

from .models import HandObservation, PicoRawFrame
from .pico import pico_to_mediapipe


# The two recent recordings require a model/PICO radial ratio of approximately
# 1.07 and 1.11. 1.10 is the stable shared value for the PICO2 acceptance
# samples; keep it explicit so a future calibration can replace one boundary
# constant without changing the official Hand2 YAML or the Manus route.
PICO_OFFICIAL_HAND2_GEOMETRY_SCALE = 1.10
PICO_OFFICIAL_HAND2_ADAPTER_VERSION = 'pico26_to_official_hand2_yflip_scale_v2'


def pico_official_hand2_retarget_input(
        keypoints_m, *, geometry_scale=PICO_OFFICIAL_HAND2_GEOMETRY_SCALE):
    """Return the PICO 21 points prepared for the official Hand2 worker.

    The scale is applied radially about the wrist (point 0), leaving the wrist
    itself unchanged. This function never mutates the canonical observation;
    only the copy returned here is submitted to the official retarget worker.
    """
    points = np.asarray(keypoints_m, dtype=np.float64)
    if points.shape != (21, 3):
        raise ValueError('official Hand2 input must have shape (21, 3)')
    if not np.isfinite(points).all():
        raise ValueError('official Hand2 input must contain only finite points')
    if isinstance(geometry_scale, (bool, np.bool_)):
        raise ValueError('official Hand2 geometry scale must be a positive finite number')
    try:
        scale = float(geometry_scale)
    except (TypeError, ValueError) as exc:
        raise ValueError('official Hand2 geometry scale must be a positive finite number') from exc
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('official Hand2 geometry scale must be a positive finite number')

    prepared = points.copy()
    prepared[1:] = prepared[0] + (prepared[1:] - prepared[0]) * scale
    return prepared


def pico_official_hand_observations(frame):
    if not isinstance(frame, PicoRawFrame):
        raise TypeError('official PICO geometry requires a PicoRawFrame')
    observations = {}
    for side in ('left', 'right'):
        hand = frame.hands[side]
        points, valid = pico_to_mediapipe([joint.pose[:3] for joint in hand.joints],
                                        [joint.valid for joint in hand.joints])
        # The official frame uses wrist/index MCP/middle MCP. Reject a
        # degenerate plane rather than sending NaNs through its SVD/normalizer.
        a, b = points[5] - points[0], points[9] - points[0]
        scale = max(float(np.max(np.abs(a))), float(np.max(np.abs(b))))
        plane_valid = False
        if scale > 1e-9:
            a, b = a / scale, b / scale
            plane_valid = bool(np.linalg.norm(np.cross(a, b)) >
                               1e-6 * np.linalg.norm(a) * np.linalg.norm(b))
        observations[side] = HandObservation(
            source='pico2', side=side,
            source_instance_id=f'{frame.receiver_instance_id}:{frame.connection_generation}',
            source_sequence=None, source_timestamp_ns=frame.source_timestamp_ns,
            received_timestamp_ns=frame.received_timestamp_ns,
            receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            coordinate_frame='pico_tracking_initial_y_reflected',
            mapping_version='pico26_to_official_hand2_yflip_v1',
            keypoints_m=points * [1., -1., 1.], joint_valid=valid,
            valid=bool(hand.valid and hand.wrist_valid and valid.all() and plane_valid),
            wrist_pose=None, frame_association_id=frame.association_id)
    return observations
