"""数据手套偏心编码器四连杆换算的公共行为测试。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from data_glove_wuji_teleop.adapters.glove.encoder_kinematics import (
    EncoderKinematics,
    FourBarLinkage,
    GloveJointFrame,
)
from data_glove_wuji_teleop.adapters.glove.encoder_stream import (
    EncoderConnection,
    EncoderFrame,
    ProtocolError,
)


def _group(
    name: str,
    channels: list[str],
    *,
    input_directions: dict[str, int],
) -> dict[str, object]:
    return {
        "name": name,
        "channels": channels,
        "input_directions": input_directions,
        "parameters": {
            "L_AD": 2.0,
            "L_AB": 1.0,
            "L_BC": 2.0,
            "L_CD": 1.0,
            "theta_A0_deg": 60.0,
            "branch": 1,
        },
    }


def _load_test_kinematics(
    *,
    length_unit: str = "millimeter",
) -> EncoderKinematics:
    config = {
        "version": 1,
        "hand": "left",
        "ready": True,
        "angle_unit": "degree",
        "length_unit": length_unit,
        "groups": [
            _group(
                "four_finger_pip",
                ["J12", "J13", "J14", "J15"],
                input_directions={
                    "J12": 1,
                    "J13": -1,
                    "J14": 1,
                    "J15": 1,
                },
            ),
            _group(
                "four_finger_dip",
                ["J17", "J18", "J19", "J20"],
                input_directions={
                    "J17": -1,
                    "J18": 1,
                    "J19": 1,
                    "J20": 1,
                },
            ),
            _group(
                "thumb_ip",
                ["J16"],
                input_directions={"J16": 1},
            ),
        ],
    }
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "encoder_kinematics.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        return EncoderKinematics.load(path)


class FourBarLinkageTest(unittest.TestCase):
    def test_zero_encoder_delta_is_zero_joint_delta(self) -> None:
        linkage = FourBarLinkage(
            L_AD=2.0,
            L_AB=1.0,
            L_BC=2.0,
            L_CD=1.0,
            theta_A0_deg=60.0,
            branch=1,
        )

        self.assertAlmostEqual(linkage.solve(0.0), 0.0)

    def test_parallelogram_has_one_to_one_input_output_motion(self) -> None:
        linkage = FourBarLinkage(
            L_AD=2.0,
            L_AB=1.0,
            L_BC=2.0,
            L_CD=1.0,
            theta_A0_deg=60.0,
            branch=1,
        )

        self.assertAlmostEqual(linkage.solve(10.0), 10.0)
        self.assertAlmostEqual(linkage.solve(-20.0), -20.0)

    def test_rejects_motion_outside_linkage_closure_range(self) -> None:
        linkage = FourBarLinkage(
            L_AD=3.0,
            L_AB=2.0,
            L_BC=2.0,
            L_CD=2.0,
            theta_A0_deg=0.0,
            branch=1,
        )

        with self.assertRaisesRegex(ValueError, "无法闭合"):
            linkage.solve(180.0)

    def test_rejects_invalid_physical_parameters(self) -> None:
        with self.assertRaisesRegex(ValueError, "杆长"):
            FourBarLinkage(
                L_AD=0.0,
                L_AB=1.0,
                L_BC=1.0,
                L_CD=1.0,
                theta_A0_deg=0.0,
            )

        with self.assertRaisesRegex(ValueError, "branch"):
            FourBarLinkage(
                L_AD=1.0,
                L_AB=1.0,
                L_BC=1.0,
                L_CD=1.0,
                theta_A0_deg=0.0,
                branch=0,
            )


class EncoderKinematicsTest(unittest.TestCase):
    def test_configuration_requires_millimeter_link_lengths(self) -> None:
        with self.assertRaisesRegex(ValueError, "length_unit.*millimeter"):
            _load_test_kinematics(length_unit="meter")

    def test_repository_config_requires_commissioning_until_ready(
        self,
    ) -> None:
        template = (
            Path(__file__).resolve().parents[2]
            / "config/dataglove/urdf/right.json"
        )
        data = json.loads(template.read_text(encoding="utf-8"))

        self.assertFalse(data["ready"])
        self.assertEqual(data["angle_unit"], "degree")
        self.assertEqual(data["length_unit"], "millimeter")
        self.assertNotIn("zero_offsets_deg", data)
        groups = {group["name"]: group for group in data["groups"]}
        self.assertEqual(
            set(groups),
            {
                "four_finger_pip",
                "four_finger_dip",
                "thumb_ip",
            },
        )
        self.assertEqual(
            groups["four_finger_pip"]["channels"],
            ["J12", "J13", "J14", "J15"],
        )
        self.assertEqual(
            groups["four_finger_dip"]["channels"],
            ["J17", "J18", "J19", "J20"],
        )
        self.assertEqual(groups["thumb_ip"]["channels"], ["J16"])

        with self.assertRaisesRegex(ValueError, "尚未填写实物参数"):
            EncoderKinematics.load(template)

        commissioning = EncoderKinematics.load(
            template,
            commissioning=True,
        )
        self.assertEqual(commissioning.unset_input_direction_channels, ())
        raw = [0.0] * 21
        raw[11] = -5.0
        provisional = commissioning.convert_angles(raw)

        explicit_data = json.loads(template.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            explicit_path = Path(directory) / "explicit-directions.json"
            explicit_data.pop("ready")
            explicit_path.write_text(
                json.dumps(explicit_data),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "尚未填写实物参数"):
                EncoderKinematics.load(explicit_path)

            explicit_data["ready"] = True
            explicit_path.write_text(
                json.dumps(explicit_data),
                encoding="utf-8",
            )
            explicit = EncoderKinematics.load(explicit_path)

        self.assertEqual(provisional, explicit.convert_angles(raw))

    def test_repository_input_directions_keep_flexion_monotonic(self) -> None:
        config = (
            Path(__file__).resolve().parents[2]
            / "config/dataglove/urdf/right.json"
        )
        kinematics = EncoderKinematics.load(config, commissioning=True)
        self.assertEqual(kinematics.unset_input_direction_channels, ())

        smaller_flexion = [0.0] * 21
        larger_flexion = [0.0] * 21
        for channel in (12, 13, 14, 15, 16):
            smaller_flexion[channel - 1] = -60.0
            larger_flexion[channel - 1] = -90.0
        for channel in (17, 18, 19, 20):
            smaller_flexion[channel - 1] = -60.0
            larger_flexion[channel - 1] = -90.0

        smaller_output = kinematics.convert_angles(smaller_flexion)
        larger_output = kinematics.convert_angles(larger_flexion)
        for channel in range(12, 21):
            self.assertGreater(smaller_output[channel - 1], 0.0)
            self.assertGreater(
                larger_output[channel - 1],
                smaller_output[channel - 1],
            )

    def test_json_groups_convert_encoder_frame_before_downstream_mapping(
        self,
    ) -> None:
        kinematics = _load_test_kinematics()

        self.assertEqual(kinematics.hand, "left")
        raw = [0.0] * 21
        raw[6] = 30.0   # J7：非四连杆通道
        raw[10] = 5.0   # J11：拇指 MCP，非四连杆通道
        raw[11] = 10.0  # J12：食指 PIP
        raw[12] = 11.0  # J13：中指 PIP，编码器安装方向相反
        raw[15] = 6.0   # J16：拇指 IP
        raw[16] = 20.0  # J17：食指 DIP
        frame = EncoderFrame(
            sequence=7,
            timestamp_ns=123,
            dropped=2,
            angles_deg=raw,
        )

        converted = kinematics.convert_zeroed_frame(frame)

        self.assertIsNot(converted, frame)
        self.assertIsInstance(converted, GloveJointFrame)
        self.assertEqual(converted.sequence, 7)
        self.assertEqual(converted.timestamp_ns, 123)
        self.assertEqual(converted.dropped, 2)
        self.assertAlmostEqual(converted.joint_angles_deg[6], 30.0)
        self.assertAlmostEqual(converted.joint_angles_deg[10], 5.0)
        self.assertAlmostEqual(converted.joint_angles_deg[11], 10.0)
        self.assertAlmostEqual(converted.joint_angles_deg[12], -11.0)
        self.assertAlmostEqual(converted.joint_angles_deg[15], 6.0)
        self.assertAlmostEqual(converted.joint_angles_deg[16], -20.0)

    def test_connection_applies_kinematics_only_to_hardware_zeroed_frame(
        self,
    ) -> None:
        kinematics = _load_test_kinematics()
        connection = EncoderConnection(None)  # type: ignore[arg-type]
        connection.channels = 21
        raw = [0.0] * 21
        raw[11] = 10.0
        frame = EncoderFrame(8, 456, 3, raw)
        with patch.object(connection, "read_frame", return_value=frame):
            with self.assertRaisesRegex(ProtocolError, "零位初始化"):
                connection.read_joint_frame(kinematics)

            with self.assertRaisesRegex(ProtocolError, "必须显式传入布尔值 True"):
                connection.read_joint_frame(
                    kinematics,
                    trust_hardware_zero=1,
                )

            converted = connection.read_joint_frame(
                kinematics,
                trust_hardware_zero=True,
            )

        self.assertIsInstance(converted, GloveJointFrame)
        self.assertEqual(converted.sequence, 8)
        self.assertAlmostEqual(converted.joint_angles_deg[11], 10.0)


if __name__ == "__main__":
    unittest.main()
