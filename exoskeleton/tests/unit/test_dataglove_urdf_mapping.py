"""实物数据手套 21 路关节帧到自身 URDF 的映射合同。"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from data_glove_wuji_teleop.adapters.glove.encoder_kinematics import (
    EncoderKinematics,
    GloveJointFrame,
)
from data_glove_wuji_teleop.profiles.dataglove.urdf_mapping import (
    DataGloveUrdfMapping,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MAPPING_PATH = (
    PROJECT_ROOT / "config/dataglove/urdf/right.json"
)
URDF_PATH = (
    PROJECT_ROOT
    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
)

EXPECTED_URDF_JOINT_NAMES = (
    "thumb_cmc_flex_joint",
    "thumb_cmc_abd_joint",
    "thumb_mcp_joint",
    "thumb_ip_joint",
    "index_mcp_abd_joint",
    "index_mcp_flex_joint",
    "index_pip_joint",
    "index_dip_joint",
    "middle_mcp_abd_joint",
    "middle_mcp_flex_joint",
    "middle_pip_joint",
    "middle_dip_joint",
    "ring_mcp_abd_joint",
    "ring_mcp_flex_joint",
    "ring_pip_joint",
    "ring_dip_joint",
    "pinky_cmc_joint",
    "pinky_mcp_abd_joint",
    "pink_mcp_flex_joint",
    "pink_pip_joint",
    "pink_dip_joint",
)

# 每个元素是对应 URDF 关节应读取的板端逻辑通道。此处故意使用独立字面量，
# 避免测试与实现共享重排常量后失去检错能力。
EXPECTED_SOURCE_CHANNELS = (
    6,
    1,
    11,
    16,
    2,
    7,
    12,
    17,
    3,
    8,
    13,
    18,
    4,
    9,
    14,
    19,
    21,
    5,
    10,
    15,
    20,
)

EXPECTED_DIRECTIONS = (
    -1, -1, 1, -1,
    -1, 1, -1, -1,
    -1, 1, -1, -1,
    -1, 1, -1, -1,
    1, -1, 1, -1, -1,
)

class DataGloveUrdfMappingTest(unittest.TestCase):
    def test_rejects_invalid_joint_output_scale(self) -> None:
        template = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        for invalid_scale in (0.0, -1.0, True, "bad", float("inf")):
            with self.subTest(scale=invalid_scale):
                data = json.loads(json.dumps(template))
                data["joints"][0]["scale"] = invalid_scale
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "invalid-scale.json"
                    path.write_text(json.dumps(data), encoding="utf-8")
                    with self.assertRaisesRegex(
                        ValueError,
                        "scale 必须是大于 0 的有限数值",
                    ):
                        DataGloveUrdfMapping.load(path)

    def test_full_skeleton_retarget_rejects_disabled_urdf_joints(self) -> None:
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        data["disabled_joint_names"] = ["ring_pip_joint"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "disabled.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            mapping = DataGloveUrdfMapping.load(path)

        with self.assertRaisesRegex(ValueError, "完整骨架.*ring_pip_joint"):
            mapping.require_all_joints_enabled()

    def test_thumb_cmc_flexion_maps_to_confirmed_urdf_direction(self) -> None:
        kinematics = EncoderKinematics.load(
            MAPPING_PATH,
            commissioning=True,
        )
        mapping = DataGloveUrdfMapping.load(MAPPING_PATH)
        raw = [0.0] * 21
        raw[5] = 30.0
        converted = kinematics.convert_angles(raw)
        urdf_frame = mapping.map_frame(
            GloveJointFrame(1, 2, 0, converted)
        )
        position_by_name = dict(
            zip(urdf_frame.joint_names, urdf_frame.positions_rad)
        )
        self.assertLess(position_by_name["thumb_cmc_flex_joint"], 0.0)

    def test_thumb_cmc_flexion_outputs_one_point_one_times_angle(
        self,
    ) -> None:
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        thumb_cmc = next(
            item
            for item in data["joints"]
            if item["joint_name"] == "thumb_cmc_flex_joint"
        )
        thumb_cmc["scale"] = 1.1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "thumb-cmc-scale.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            mapping = DataGloveUrdfMapping.load(path)
        actual_angles_deg = [0.0] * 21
        actual_angles_deg[5] = 10.0

        urdf_frame = mapping.map_frame(
            GloveJointFrame(1, 2, 0, tuple(actual_angles_deg))
        )
        position_by_name = dict(
            zip(urdf_frame.joint_names, urdf_frame.positions_rad)
        )

        self.assertAlmostEqual(
            math.degrees(position_by_name["thumb_cmc_flex_joint"]),
            -11.0,
        )

    def test_physical_thumb_ip_flexion_maps_to_confirmed_urdf_direction(
        self,
    ) -> None:
        kinematics = EncoderKinematics.load(
            MAPPING_PATH,
            commissioning=True,
        )
        mapping = DataGloveUrdfMapping.load(MAPPING_PATH)
        raw = [0.0] * 21
        raw[15] = -60.0
        converted = kinematics.convert_angles(raw)
        urdf_frame = mapping.map_frame(
            GloveJointFrame(1, 2, 0, converted)
        )
        position_by_name = dict(
            zip(urdf_frame.joint_names, urdf_frame.positions_rad)
        )
        self.assertLess(position_by_name["thumb_ip_joint"], 0.0)

    def test_physical_pip_flexion_maps_to_confirmed_urdf_direction(self) -> None:
        kinematics = EncoderKinematics.load(
            MAPPING_PATH,
            commissioning=True,
        )
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        data["disabled_joint_names"] = []
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "all-outputs-enabled.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            mapping = DataGloveUrdfMapping.load(path)
        raw = [0.0] * 21
        for channel in (12, 13, 14, 15):
            raw[channel - 1] = -60.0
        converted = kinematics.convert_angles(raw)
        urdf_frame = mapping.map_frame(
            GloveJointFrame(1, 2, 0, converted)
        )
        position_by_name = dict(
            zip(urdf_frame.joint_names, urdf_frame.positions_rad)
        )
        for name in (
            "index_pip_joint",
            "middle_pip_joint",
            "ring_pip_joint",
            "pink_pip_joint",
        ):
            self.assertLess(position_by_name[name], 0.0)

    def test_four_finger_mcp_flexion_outputs_one_point_five_times_angle(
        self,
    ) -> None:
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        data["disabled_joint_names"] = []
        mcp_flex_names = {
            "index_mcp_flex_joint",
            "middle_mcp_flex_joint",
            "ring_mcp_flex_joint",
            "pink_mcp_flex_joint",
        }
        for item in data["joints"]:
            if item["joint_name"] in mcp_flex_names:
                item["scale"] = 1.5
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "all-outputs-enabled.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            mapping = DataGloveUrdfMapping.load(path)
        actual_angles_deg = [0.0] * 21
        for channel in (7, 8, 9, 10):
            actual_angles_deg[channel - 1] = 20.0

        urdf_frame = mapping.map_frame(
            GloveJointFrame(1, 2, 0, tuple(actual_angles_deg))
        )
        position_by_name = dict(
            zip(urdf_frame.joint_names, urdf_frame.positions_rad)
        )

        for name in (
            "index_mcp_flex_joint",
            "middle_mcp_flex_joint",
            "ring_mcp_flex_joint",
            "pink_mcp_flex_joint",
        ):
            self.assertAlmostEqual(
                math.degrees(position_by_name[name]),
                30.0,
                msg=name,
            )

    def test_physical_dip_flexion_does_not_stall_or_reverse(self) -> None:
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        data["disabled_joint_names"] = []
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "all-outputs-enabled.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            kinematics = EncoderKinematics.load(path, commissioning=True)
            mapping = DataGloveUrdfMapping.load(path)

        dip_names = (
            "index_dip_joint",
            "middle_dip_joint",
            "ring_dip_joint",
            "pink_dip_joint",
        )
        output_by_name = {name: [] for name in dip_names}
        flexion_samples = (
            0.0,
            -20.0,
            -40.0,
            -60.0,
            -80.0,
            -100.0,
            -110.0,
        )
        for flexion_deg in flexion_samples:
            raw = [0.0] * 21
            for channel in (17, 18, 19, 20):
                raw[channel - 1] = flexion_deg
            converted = kinematics.convert_angles(raw)
            urdf_frame = mapping.map_frame(
                GloveJointFrame(1, 2, 0, converted)
            )
            positions = dict(
                zip(urdf_frame.joint_names, urdf_frame.positions_rad)
            )
            for name in dip_names:
                output_by_name[name].append(positions[name])

        for name, positions in output_by_name.items():
            for shallower, deeper in zip(positions, positions[1:]):
                self.assertLess(deeper, shallower, name)
            self.assertLess(
                math.degrees(positions[-1]),
                -75.0,
                name,
            )

    def test_repository_config_covers_hardware_and_urdf_joint_order(self) -> None:
        mapping = DataGloveUrdfMapping.load(MAPPING_PATH)
        self.assertEqual(mapping.hand, "right")
        mapping.validate_urdf(URDF_PATH)
        self.assertFalse(mapping.directions_verified)
        with self.assertRaisesRegex(ValueError, "方向尚未完成实物验证"):
            mapping.require_verified_directions()
        mapping.validate_stream(
            channels=21,
            cs_by_joint=(
                18, 20, 19, 15, 11, 3, 14, 10, 6, 2, 17,
                13, 9, 5, 1, 16, 12, 8, 4, 0, 7,
            ),
        )
        with self.assertRaisesRegex(ValueError, "配置=.*板端="):
            mapping.validate_stream(
                channels=21,
                cs_by_joint=range(21),
            )
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            tuple(
                (item["joint_name"], item["source_channel"])
                for item in data["joints"]
            ),
            tuple(
                (joint_name, f"J{source_channel}")
                for joint_name, source_channel in zip(
                    EXPECTED_URDF_JOINT_NAMES,
                    EXPECTED_SOURCE_CHANNELS,
                )
            ),
        )
        self.assertEqual(
            tuple(item["direction"] for item in data["joints"]),
            EXPECTED_DIRECTIONS,
        )

    def test_maps_configured_directions_and_preserves_frame_metadata(self) -> None:
        data = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
        directions = tuple(-1 if index % 2 == 0 else 1 for index in range(21))
        data["directions_verified"] = True
        data["disabled_joint_names"] = []
        for item, direction in zip(data["joints"], directions):
            item["direction"] = direction
            item["scale"] = 1.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "dataglove_urdf.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            mapping = DataGloveUrdfMapping.load(path)
        mapping.require_verified_directions()
        frame = GloveJointFrame(
            sequence=42,
            timestamp_ns=123_456,
            dropped=3,
            joint_angles_deg=tuple(float(index) for index in range(1, 22)),
        )

        urdf_frame = mapping.map_frame(frame)

        self.assertEqual(urdf_frame.sequence, 42)
        self.assertEqual(urdf_frame.timestamp_ns, 123_456)
        self.assertEqual(urdf_frame.dropped, 3)
        self.assertEqual(urdf_frame.joint_names, EXPECTED_URDF_JOINT_NAMES)
        self.assertEqual(
            tuple(round(math.degrees(value)) for value in urdf_frame.positions_rad),
            tuple(
                direction * source_channel
                for direction, source_channel in zip(
                    directions,
                    EXPECTED_SOURCE_CHANNELS,
                )
            ),
        )
        self.assertEqual(
            round(math.degrees(urdf_frame.positions_rad[16])),
            -21,
            "J21 必须驱动 pinky_cmc_joint，不能被 20-DOF 路径丢弃",
        )


if __name__ == "__main__":
    unittest.main()
