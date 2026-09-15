"""真实 Wuji 模型命名、限位和本机隔离 UDP 数据报合同。"""

from __future__ import annotations

import struct
import math
import random
import socket
import time
import zlib
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from data_glove_wuji_teleop.adapters.transport.tianji_udp import (
    TianjiHandSender,
    TianjiJointBinding,
)
from data_glove_wuji_teleop.profiles.wuji_v2.model import full_joint_name
from data_glove_wuji_teleop.project import get_hand_resources


# 独立记录协议槽位，避免生产常量写错时测试仍然自证正确。
PROTOCOL_NAMES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)


def decode_packet(payload: bytes) -> dict[str, object]:
    """独立按控制器线格式解包，不借用生产编码器或常量。"""
    assert len(payload) == 364
    assert struct.unpack("<I", payload[-4:])[0] == zlib.crc32(payload[:-4]) & 0xFFFFFFFF
    magic, version, flags, size, sequence, published, left_ns, right_ns, *angles = struct.unpack(
        "<4sBBHQqqq40d", payload[:-4],
    )
    assert (magic, version, size) == (b"TJH2", 2, 364)
    assert flags in (1, 2, 3)
    return {
        "flags": flags, "sequence": sequence, "timestamp_ns": published,
        "left_timestamp_ns": left_ns, "right_timestamp_ns": right_ns,
        "left": tuple(angles[:20]), "right": tuple(angles[20:]),
    }


def load_real_model(hand: str) -> mujoco.MjModel:
    resources = get_hand_resources(hand, generation="v2")
    return mujoco.MjModel.from_xml_path(str(resources.mjcf))


def model_names(hand: str) -> tuple[str, ...]:
    return tuple(full_joint_name(hand, name) for name in PROTOCOL_NAMES)


def shuffled_result(
    hand: str, model: mujoco.MjModel,
) -> tuple[TianjiJointBinding, np.ndarray, tuple[float, ...]]:
    """构造具备正负号且可区分槽位的真实模型范围内 qpos。"""

    names = list(model_names(hand))
    expected = tuple((index + 1) / 100 * (-1 if (index + (hand == "right")) % 2 else 1)
                     for index in range(20))
    by_name = dict(zip(names, expected, strict=True))
    random.Random(131 if hand == "left" else 927).shuffle(names)
    qpos = np.array([by_name[name] for name in names], dtype=np.float64)
    return TianjiJointBinding.from_model(hand, names, model), qpos, expected


class TianjiJointBindingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.models = {hand: load_real_model(hand) for hand in ("left", "right")}

    def test_shuffled_robot_qpos_preserves_both_hands_signs_and_scale(self) -> None:
        for hand, model in self.models.items():
            with self.subTest(hand=hand):
                binding, qpos, expected = shuffled_result(hand, model)
                self.assertEqual(binding.convert(qpos), expected)
                # 拇指末两段和食指均有非零输入；重套1.2/1.5/-1会破坏上述相等。
                self.assertNotEqual(binding.source_joint_names, model_names(hand))

    def test_rejects_missing_unknown_opposite_and_duplicate_names(self) -> None:
        names = model_names("left")
        invalid = (
            names[:-1],
            ("l_unknown", *names[1:]),
            (full_joint_name("right", PROTOCOL_NAMES[0]), *names[1:]),
            (names[1], *names[1:]),
        )
        for candidate in invalid:
            with self.subTest(names=candidate):
                with self.assertRaises(ValueError):
                    TianjiJointBinding.from_model("left", candidate, self.models["left"])
        with self.assertRaises(ValueError):
            TianjiJointBinding.from_model("left", names, self.models["right"])
        with self.assertRaises(ValueError):
            TianjiJointBinding.from_model("both", names, self.models["left"])

    def test_rejects_nonfinite_nonnumeric_and_wrong_shape_qpos(self) -> None:
        binding, qpos, _ = shuffled_result("left", self.models["left"])
        for bad in (math.nan, math.inf, -math.inf, True, np.bool_(False), "0.1"):
            values = list(qpos)
            values[4] = bad
            with self.subTest(value=bad), self.assertRaises(ValueError):
                binding.convert(values)
        for values in (qpos[:-1], np.zeros((20, 1))):
            with self.subTest(shape=np.shape(values)), self.assertRaises(ValueError):
                binding.convert(values)

    def test_model_endpoints_and_outside_angles_are_forwarded_without_clipping(self) -> None:
        for hand, model in self.models.items():
            names = model_names(hand)
            binding = TianjiJointBinding.from_model(hand, names, model)
            for slot, name in enumerate(names):
                joint_id = model.joint(name).id
                actuator_id = next(i for i in range(model.nu)
                                   if model.actuator_trnid[i, 0] == joint_id)
                lower = max(model.jnt_range[joint_id, 0], model.actuator_ctrlrange[actuator_id, 0])
                upper = min(model.jnt_range[joint_id, 1], model.actuator_ctrlrange[actuator_id, 1])
                for endpoint, direction in ((lower, -math.inf), (upper, math.inf)):
                    values = [0.0] * 20
                    values[slot] = float(endpoint)
                    with self.subTest(hand=hand, name=name, endpoint=endpoint):
                        self.assertEqual(binding.convert(values), tuple(values))
                        for outside in (math.nextafter(float(endpoint), direction),
                                        float(endpoint) + math.copysign(0.5, direction)):
                            values[slot] = outside
                            self.assertEqual(binding.convert(values), tuple(values))

    def test_rejects_invalid_limits_and_nonposition_actuator(self) -> None:
        # 修改真实模型的已编译属性，覆盖失去安全合同的模型配置。
        cases = (
            ("jnt_limited", 0, 0),
            ("actuator_ctrllimited", 0, 0),
            ("jnt_range", 0, (math.nan, 1.0)),
            ("actuator_ctrlrange", 0, (-1.0, math.inf)),
            ("jnt_range", 0, (0.5, -0.5)),
            ("actuator_ctrlrange", 0, (2.0, 3.0)),
            ("actuator_biasprm", (0, 1), 0.0),
            ("actuator_gear", (0, 0), 2.0),
            ("actuator_trnid", (0, 0), 1),
        )
        for field, index, value in cases:
            with self.subTest(field=field, value=value):
                model = load_real_model("left")
                getattr(model, field)[index] = value
                with self.assertRaises(ValueError):
                    TianjiJointBinding.from_model("left", model_names("left"), model)


class TianjiHandSenderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.angles = {}
        for hand in ("left", "right"):
            binding, qpos, expected = shuffled_result(hand, load_real_model(hand))
            cls.angles[hand] = binding.convert(qpos)
            if cls.angles[hand] != expected:
                raise AssertionError("真实模型 qpos 与协议槽位不符")

    def setUp(self) -> None:
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.receiver.close)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(0.05)
        self.port = self.receiver.getsockname()[1]
        self.sender = TianjiHandSender("127.0.0.1", self.port)
        self.addCleanup(self.sender.close)

    def receive(self) -> dict[str, object]:
        payload, _ = self.receiver.recvfrom(8192)
        return decode_packet(payload)

    def assert_no_packet(self) -> None:
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)

    def test_asymmetric_updates_preserve_other_side_pose_and_sample_age(self) -> None:
        left_ns = time.monotonic_ns()
        first = self.sender.send(left=self.angles["left"], left_timestamp_ns=left_ns)
        left = self.receive()
        self.assertEqual(left["flags"], 1)
        self.assertEqual(left["left"], self.angles["left"])
        self.assertEqual(left["right"], (0.0,) * 20)
        self.assertEqual(left["right_timestamp_ns"], 0)
        right_ns = time.monotonic_ns()
        second = self.sender.send(right=self.angles["right"], right_timestamp_ns=right_ns)
        both = self.receive()
        self.assertEqual(both["flags"], 3)
        self.assertEqual(both["left"], self.angles["left"])
        self.assertEqual(both["right"], self.angles["right"])
        self.assertEqual(both["left_timestamp_ns"], left_ns)
        self.assertEqual(both["right_timestamp_ns"], right_ns)
        self.assertGreater(second, first)
        self.assertGreaterEqual(both["timestamp_ns"], right_ns)
        self.assertIsNone(self.sender.send())
        self.assert_no_packet()

    def test_invalidation_removes_failed_cached_side_without_sending(self) -> None:
        self.sender.send(left=self.angles["left"], left_timestamp_ns=time.monotonic_ns())
        self.receive()
        self.sender.invalidate("left")
        self.assert_no_packet()
        self.sender.send(right=self.angles["right"], right_timestamp_ns=time.monotonic_ns())
        message = self.receive()
        self.assertEqual(message["flags"], 2)
        self.assertEqual(message["left"], (0.0,) * 20)
        self.assertEqual(message["left_timestamp_ns"], 0)
        self.assertEqual(message["right"], self.angles["right"])

    def test_empty_send_and_close_never_send_exit_pose(self) -> None:
        self.assertIsNone(self.sender.send())
        self.assert_no_packet()
        self.sender.send(left=self.angles["left"], left_timestamp_ns=time.monotonic_ns())
        self.receive()
        self.sender.close()
        self.sender.close()
        self.assertIsNone(self.sender.send())
        with self.assertRaises(RuntimeError):
            self.sender.send(right=self.angles["right"], right_timestamp_ns=time.monotonic_ns())
        self.assert_no_packet()

    def test_malformed_bilateral_update_does_not_replace_cached_valid_side(self) -> None:
        original_ns = time.monotonic_ns()
        self.sender.send(left=self.angles["left"], left_timestamp_ns=original_ns)
        self.receive()
        changed = (0.9,) * 20
        for bad in (True, np.bool_(False), "0.2", math.nan, math.inf, -math.inf, 10**1000):
            invalid = list(self.angles["right"])
            invalid[7] = bad
            with self.subTest(value=bad), self.assertRaises(ValueError):
                self.sender.send(left=changed, right=invalid,
                                 left_timestamp_ns=time.monotonic_ns(),
                                 right_timestamp_ns=time.monotonic_ns())
        for invalid in ([], [0.0] * 19, [0.0] * 21, np.zeros((20, 1))):
            with self.assertRaises(ValueError):
                self.sender.send(left=changed, right=invalid,
                                 left_timestamp_ns=time.monotonic_ns(),
                                 right_timestamp_ns=time.monotonic_ns())
        for bad_timestamp in (0, -1, True, 1.5, 2**63):
            with self.subTest(timestamp=bad_timestamp), self.assertRaises(ValueError):
                self.sender.send(left=changed, right=self.angles["right"],
                                 left_timestamp_ns=time.monotonic_ns(),
                                 right_timestamp_ns=bad_timestamp)
        self.assert_no_packet()
        self.sender.send(right=self.angles["right"], right_timestamp_ns=time.monotonic_ns())
        message = self.receive()
        self.assertEqual(message["left"], self.angles["left"])
        self.assertEqual(message["left_timestamp_ns"], original_ns)

    def test_monotonic_sequence_survives_repeated_clock_ticks(self) -> None:
        epoch = 1_000_000_123
        with patch("data_glove_wuji_teleop.adapters.transport.tianji_udp.time.monotonic_ns",
                   side_effect=(epoch, epoch, epoch, epoch + 5000)):
            sender = TianjiHandSender("127.0.0.1", self.port)
            self.addCleanup(sender.close)
            for side, expected in (("left", epoch + 1), ("right", epoch + 2), ("left", epoch + 5000)):
                sequence = sender.send(**{side: self.angles[side], f"{side}_timestamp_ns": epoch})
                self.assertEqual(sequence, expected)
                self.assertEqual(self.receive()["sequence"], expected)

    def test_rejects_nonloopback_and_bad_port_before_opening_socket(self) -> None:
        with patch("socket.socket", side_effect=AssertionError("不能创建套接字")):
            for host in ("192.168.1.2", "0.0.0.0", "8.8.8.8", "localhost", "::1",
                         "127.1", "256.0.0.1", "", 123):
                with self.subTest(host=host), self.assertRaises(ValueError):
                    TianjiHandSender(host, self.port)
            for port in (0, -1, 65536, True, 16000.0, "16000"):
                with self.subTest(port=port), self.assertRaises(ValueError):
                    TianjiHandSender("127.0.0.1", port)


if __name__ == "__main__":
    unittest.main()
