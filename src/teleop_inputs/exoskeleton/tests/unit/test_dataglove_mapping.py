#!/usr/bin/env python3
"""数据手套到 Wuji 关节目标的公共映射合同测试。"""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from data_glove_wuji_teleop.profiles.dataglove.mapping import (
    AxisCalibration,
    CalibrationProfile,
    apply_faulty_encoder_reuse,
    build_profile_from_captures,
    shortest_angular_delta_deg,
)


class CalibrationProfileMappingTest(unittest.TestCase):
    def test_maps_encoder_motion_across_zero_degree_boundary(self) -> None:
        self.assertAlmostEqual(
            shortest_angular_delta_deg(340.0, 80.0),
            -100.0,
        )
        zero_offsets = [0.0] * 21
        zero_offsets[5] = 80.0
        profile = CalibrationProfile.from_dict(
            {
                "version": 1,
                "hand": "left",
                "channels": 21,
                "zero_offsets_deg": zero_offsets,
                "cs_by_joint": list(range(21)),
                "axes": {
                    "J6": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -100.0,
                            "output_rad": 1.0,
                        },
                    },
                },
            }
        )

        angles = zero_offsets.copy()
        angles[5] = 340.0
        self.assertAlmostEqual(
            profile.map_to_finger_target(angles)[0][0],
            1.0,
        )

    def test_rejects_unwrapped_calibration_endpoint(self) -> None:
        with self.assertRaisesRegex(ValueError, "最短圆周角差"):
            CalibrationProfile.from_dict(
                {
                    "version": 1,
                    "hand": "left",
                    "channels": 21,
                    "zero_offsets_deg": [0.0] * 21,
                    "cs_by_joint": list(range(21)),
                    "axes": {
                        "J6": {
                            "enabled": True,
                            "positive": {
                                "input_deg": 256.0,
                                "output_rad": 1.0,
                            },
                        },
                    },
                }
            )

    def test_direct_profile_construction_rejects_invalid_input_sources(
        self,
    ) -> None:
        with self.assertRaisesRegex(ValueError, "输入源.*20"):
            CalibrationProfile(
                hand="left",
                cs_by_joint=tuple(range(21)),
                axes=(AxisCalibration(),) * 20,
                input_source_indices=(0,),
            )

    def test_faulty_encoder_reuse_preserves_destination_scales(
        self,
    ) -> None:
        zero_offsets = [0.0] * 21
        zero_offsets[8] = 10.0
        zero_offsets[13] = 20.0
        zero_offsets[15] = 100.0
        zero_offsets[18] = 30.0
        profile = CalibrationProfile.from_dict(
            {
                "version": 1,
                "hand": "left",
                "channels": 21,
                "zero_offsets_deg": zero_offsets,
                "cs_by_joint": list(range(21)),
                "axes": {
                    "J1": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -20.0,
                            "output_rad": 0.2,
                        },
                    },
                    "J6": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "positive": {
                            "input_deg": 40.0,
                            "output_rad": -0.4,
                        },
                    },
                    "J9": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -50.0,
                            "output_rad": 0.8,
                        },
                    },
                    "J14": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -60.0,
                            "output_rad": 0.9,
                        },
                    },
                    "J16": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -100.0,
                            "output_rad": 1.4,
                        },
                    },
                    "J19": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -30.0,
                            "output_rad": 1.0,
                        },
                    },
                },
            }
        )

        reused = apply_faulty_encoder_reuse(profile)
        angles = zero_offsets.copy()
        angles[0] = -10.0
        angles[5] = -999.0
        angles[8] = -40.0
        angles[13] = -40.0
        angles[15] = 50.0
        angles[18] = 0.0
        target = reused.map_to_finger_target(angles)

        self.assertAlmostEqual(target[0][0], -0.2)
        self.assertAlmostEqual(target[0][1], 0.1)
        self.assertAlmostEqual(target[0][3], 0.7)
        self.assertEqual(target[3], [0.8, 0.0, 0.9, 1.0])
        self.assertEqual(target[4], target[3])
        self.assertFalse(reused.axes[4].enabled)
        self.assertEqual(reused.axes[0], profile.axes[0])
        self.assertEqual(
            reused.to_dict()["input_sources"],
            {
                "J6": "J16",
                "J10": "J9",
                "J15": "J14",
                "J20": "J19",
            },
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left.json"
            reused.save(path)
            loaded = CalibrationProfile.load(path)
        self.assertEqual(loaded.map_to_finger_target(angles), target)

    def test_profile_persists_and_reads_target_from_reused_input_channel(
        self,
    ) -> None:
        zero_offsets = [0.0] * 21
        zero_offsets[15] = 100.0
        profile = CalibrationProfile.from_dict(
            {
                "version": 1,
                "hand": "left",
                "channels": 21,
                "zero_offsets_deg": zero_offsets,
                "input_sources": {"J6": "J16"},
                "cs_by_joint": list(range(21)),
                "axes": {
                    "J6": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "negative": {
                            "input_deg": -50.0,
                            "output_rad": 0.6,
                        },
                    },
                },
            }
        )

        angles = zero_offsets.copy()
        angles[5] = -999.0
        angles[15] = 75.0
        self.assertAlmostEqual(
            profile.map_to_finger_target(angles)[0][0],
            0.3,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left.json"
            profile.save(path)
            loaded = CalibrationProfile.load(path)
        self.assertEqual(
            loaded.to_dict()["input_sources"],
            {"J6": "J16"},
        )
        self.assertEqual(loaded.to_dict(), profile.to_dict())

    def test_profile_persists_and_subtracts_calibrated_zero(self) -> None:
        zero_offsets = [0.0] * 21
        zero_offsets[0] = 20.0
        zero_offsets[5] = 5.0
        profile = CalibrationProfile.from_dict(
            {
                "version": 1,
                "hand": "right",
                "channels": 21,
                "zero_offsets_deg": zero_offsets,
                "cs_by_joint": list(range(21)),
                "axes": {
                    "J1": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "positive": {
                            "input_deg": 10.0,
                            "output_rad": 0.2,
                        },
                    },
                    "J6": {
                        "enabled": True,
                        "deadband_deg": 0.0,
                        "positive": {
                            "input_deg": 30.0,
                            "output_rad": 1.0,
                        },
                    },
                },
            }
        )

        self.assertEqual(
            profile.map_to_finger_target(zero_offsets)[0],
            [0.0] * 4,
        )
        moved = zero_offsets.copy()
        moved[0] = 30.0
        moved[5] = 35.0
        self.assertEqual(
            profile.map_to_finger_target(moved)[0],
            [1.0, 0.2, 0.0, 0.0],
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "right.json"
            profile.save(path)
            loaded = CalibrationProfile.load(path)
        self.assertEqual(loaded.to_dict(), profile.to_dict())
        self.assertEqual(
            loaded.to_dict()["zero_offsets_deg"],
            zero_offsets,
        )

    def test_maps_j_order_to_wuji_matrix_and_existing_dof_order(self) -> None:
        axes = {
            f"J{index}": {
                "enabled": index != 3,
                "deadband_deg": 0.0,
                "positive": {
                    "input_deg": 1.0,
                    "output_rad": float(index),
                },
            }
            for index in range(1, 21)
        }
        profile = CalibrationProfile.from_dict(
            {
                "version": 1,
                "hand": "right",
                "channels": 21,
                "cs_by_joint": list(range(21)),
                "axes": axes,
            }
        )

        target = profile.map_to_finger_target([1.0] * 21)
        dof = profile.map_to_dof_values([1.0] * 20 + [999.0])

        self.assertEqual(
            target,
            [
                [6.0, 1.0, 11.0, 16.0],
                [7.0, 2.0, 12.0, 17.0],
                [8.0, 0.0, 13.0, 18.0],
                [9.0, 4.0, 14.0, 19.0],
                [10.0, 5.0, 15.0, 20.0],
            ],
        )
        self.assertEqual(
            dof,
            [
                2.0,
                7.0,
                12.0,
                17.0,
                0.0,
                8.0,
                13.0,
                18.0,
                4.0,
                9.0,
                14.0,
                19.0,
                5.0,
                10.0,
                15.0,
                20.0,
                6.0,
                1.0,
                11.0,
                16.0,
            ],
        )
        self.assertTrue(all(math.isfinite(value) for value in dof))

    def test_builds_deadbanded_flex_and_two_sided_lateral_scales(self) -> None:
        disabled = {
            *(f"J{index}" for index in range(2, 6)),
            *(f"J{index}" for index in range(7, 21)),
        }
        profile = build_profile_from_captures(
            hand="right",
            cs_by_joint=range(21),
            flex_endpoints_deg={"J6": -41.0},
            flex_targets_rad={"J6": 1.0},
            lateral_ranges_deg={"J1": (-21.0, 11.0)},
            lateral_targets_rad={"J1": (-0.2, 0.3)},
            disabled_channels=disabled,
            deadband_deg=1.0,
        )

        angles = [0.0] * 21
        angles[0] = -11.0
        angles[5] = -21.0
        target = profile.map_to_finger_target(angles)

        self.assertAlmostEqual(target[0][0], 0.5)
        self.assertAlmostEqual(target[0][1], -0.1)

        angles[0] = 6.0
        angles[5] = -100.0
        target = profile.map_to_finger_target(angles)
        self.assertAlmostEqual(target[0][0], 1.0)
        self.assertAlmostEqual(target[0][1], 0.15)

        angles[0] = 0.5
        angles[5] = 0.5
        self.assertEqual(profile.map_to_finger_target(angles)[0], [0.0] * 4)

    def test_builds_one_sided_lateral_scale(self) -> None:
        disabled = {
            *(f"J{index}" for index in range(2, 6)),
            *(f"J{index}" for index in range(6, 21)),
        }
        profile = build_profile_from_captures(
            hand="left",
            cs_by_joint=range(21),
            flex_endpoints_deg={},
            flex_targets_rad={},
            lateral_ranges_deg={"J1": (0.0, 16.0)},
            lateral_targets_rad={"J1": (-0.2, 0.3)},
            disabled_channels=disabled,
        )

        axis = profile.axes[0]
        self.assertIsNone(axis.negative)
        self.assertIsNotNone(axis.positive)
        self.assertEqual(axis.map(-20.0), 0.0)
        self.assertAlmostEqual(axis.map(8.0), 0.15)
        self.assertAlmostEqual(axis.map(20.0), 0.3)

    def test_stream_guard_checks_channels_and_mapping(self) -> None:
        disabled = {f"J{index}" for index in range(1, 21)}
        profile = build_profile_from_captures(
            hand="left",
            cs_by_joint=range(21),
            flex_endpoints_deg={},
            flex_targets_rad={},
            lateral_ranges_deg={},
            lateral_targets_rad={},
            disabled_channels=disabled,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left.json"
            profile.save(path)
            loaded = CalibrationProfile.load(path)

        self.assertEqual(loaded.to_dict(), profile.to_dict())
        loaded.validate_stream(
            channels=21,
            cs_by_joint=range(21),
        )
        with self.assertRaisesRegex(ValueError, "enc_map"):
            loaded.validate_stream(
                channels=21,
                cs_by_joint=reversed(range(21)),
            )


if __name__ == "__main__":
    unittest.main()
