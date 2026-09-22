"""SDK model failures must fail closed rather than publish plausible commands."""

from types import SimpleNamespace as Obj

import numpy as np
import pytest

from manus_bridge.hand2_sdk import HAND2_JOINT_LABELS, WujiHand2Retargeter


def retargeter_returning(values):
    # The fake is deliberately only the IK surface; there is no device API.
    sdk = Obj(
        Handedness=Obj(Left="left", Right="right"), HandModel=Obj(WujiHand2="hand2"),
        RetargetSession=Obj(for_hand=lambda *args, **kwargs: Obj(
            step=lambda points: values, reset=lambda: None,
        )),
    )
    return WujiHand2Retargeter("left", sdk=sdk)


def test_named_firmware_limits_allow_only_roundoff_at_endpoints():
    values = np.zeros(20, dtype=np.float32)
    # Thumb S2 and index S2 have different lower limits; exchanging either
    # axis/order must not reinterpret a safe firmware-order command.
    values[1] = -1.484 - 0.00005
    values[5] = 0.698 + 0.00005
    result = retargeter_returning(values).retarget(np.ones((21, 3)))
    command = dict(zip(HAND2_JOINT_LABELS, result, strict=True))
    assert tuple(command) == (
        "thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4",
        "index_S1", "index_S2", "index_S3", "index_S4",
        "middle_S1", "middle_S2", "middle_S3", "middle_S4",
        "ring_S1", "ring_S2", "ring_S3", "ring_S4",
        "pinky_S1", "pinky_S2", "pinky_S3", "pinky_S4",
    )
    assert command["thumb_S2"] == pytest.approx(-1.484)
    assert command["index_S2"] == pytest.approx(0.698)
    values[5] = 0.699
    with pytest.raises(RuntimeError):
        retargeter_returning(values).retarget(np.ones((21, 3)))


@pytest.mark.parametrize("values", [np.zeros(19), np.full(20, np.nan)])
def test_malformed_sdk_result_is_never_a_command(values):
    with pytest.raises(RuntimeError):
        retargeter_returning(values).retarget(np.ones((21, 3)))
