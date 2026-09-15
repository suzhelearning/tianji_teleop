"""实物数据手套 URDF 逐指软件零位标定合同。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from data_glove_wuji_teleop.application.urdf_zero_calibration import (
    command_calibrate_urdf_zero,
)
from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import (
    URDF_ZERO_GROUPS,
    UrdfZeroProfile,
)


EXPECTED_GROUP_CHANNELS = (
    ("thumb", ("J1", "J6", "J11", "J16")),
    ("index", ("J2", "J7", "J12", "J17")),
    ("middle", ("J3", "J8", "J13", "J18")),
    ("ring", ("J4", "J9", "J14", "J19")),
    ("pinky", ("J5", "J10", "J15", "J20")),
    ("pinky_cmc", ("J21",)),
)


def frame(sequence: int, angles: list[float]) -> EncoderFrame:
    return EncoderFrame(
        sequence=sequence,
        timestamp_ns=sequence * 10_000_000,
        dropped=0,
        angles_deg=angles,
    )


def angle_rows(frames: list[EncoderFrame]) -> list[list[float]]:
    return [item.angles_deg for item in frames]


class UrdfZeroProfileTest(unittest.TestCase):
    def test_invalid_numeric_field_is_reported_as_profile_value_error(
        self,
    ) -> None:
        with self.assertRaisesRegex(ValueError, "version"):
            UrdfZeroProfile.from_dict({"version": None})

    def test_six_steps_cover_five_digits_then_pinky_cmc_exactly_once(
        self,
    ) -> None:
        self.assertEqual(
            tuple((group.name, group.channels) for group in URDF_ZERO_GROUPS),
            EXPECTED_GROUP_CHANNELS,
        )
        self.assertEqual(
            sorted(
                channel
                for group in URDF_ZERO_GROUPS
                for channel in group.channels
            ),
            sorted(f"J{index}" for index in range(1, 22)),
        )

    def test_capture_updates_only_selected_group_and_handles_angle_wrap(
        self,
    ) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))
        frames = []
        for sequence, value in enumerate((359.0, 0.0, 1.0), start=1):
            angles = [200.0] * 21
            for channel in (1, 6, 11, 16):
                angles[channel - 1] = value
            frames.append(frame(sequence, angles))

        captured, summary = profile.capture_group(
            "thumb",
            angle_rows(frames),
            max_motion_deg=3.0,
        )

        self.assertEqual(summary.group.name, "thumb")
        self.assertEqual(captured.captured_groups, ("thumb",))
        for channel in (1, 6, 11, 16):
            self.assertAlmostEqual(
                captured.zero_offsets_deg[channel - 1],
                0.0,
            )
        for channel in set(range(1, 22)) - {1, 6, 11, 16}:
            self.assertIsNone(captured.zero_offsets_deg[channel - 1])

    def test_unstable_selected_channel_is_rejected_without_partial_update(
        self,
    ) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))
        frames = []
        for sequence, value in enumerate((10.0, 30.0, 50.0), start=1):
            angles = [0.0] * 21
            angles[1] = value
            frames.append(frame(sequence, angles))

        with self.assertRaisesRegex(ValueError, "J2.*不稳定"):
            profile.capture_group(
                "index",
                angle_rows(frames),
                max_motion_deg=2.0,
            )

        self.assertEqual(profile.captured_groups, ())
        self.assertTrue(all(value is None for value in profile.zero_offsets_deg))

    def test_complete_profile_applies_shortest_signed_delta(
        self,
    ) -> None:
        profile = UrdfZeroProfile.from_dict(
            {
                "version": 1,
                "hand": "right",
                "channels": 21,
                "angle_unit": "degree",
                "stream_zeroed": False,
                "stream_range_deg": [0.0, 360.0],
                "expected_cs_by_joint": list(range(21)),
                "zero_offsets_deg": {
                    f"J{index}": 350.0 for index in range(1, 22)
                },
                "captured_groups": [
                    "thumb",
                    "index",
                    "middle",
                    "ring",
                    "pinky",
                    "pinky_cmc",
                ],
            }
        )
        zeroed = profile.apply_angles([10.0] * 21)

        self.assertEqual(zeroed, [20.0] * 21)

    def test_profile_preserves_positive_half_turn_direction(self) -> None:
        profile = UrdfZeroProfile.from_dict(
            {
                "version": 1,
                "hand": "right",
                "channels": 21,
                "angle_unit": "degree",
                "stream_zeroed": False,
                "stream_range_deg": [0.0, 360.0],
                "expected_cs_by_joint": list(range(21)),
                "zero_offsets_deg": {
                    f"J{index}": 0.0 for index in range(1, 22)
                },
                "captured_groups": [
                    "thumb",
                    "index",
                    "middle",
                    "ring",
                    "pinky",
                    "pinky_cmc",
                ],
            }
        )

        self.assertEqual(profile.apply_angles([180.0] * 21), [180.0] * 21)

    def test_incomplete_profile_cannot_drive_urdf(self) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))

        with self.assertRaisesRegex(ValueError, "零位标定未完成"):
            profile.apply_angles([0.0] * 21)

    def test_profile_round_trip_preserves_partial_progress(self) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))
        profile, _ = profile.capture_group(
            "pinky_cmc",
            [[21.0] * 21, [21.0] * 21],
            max_motion_deg=1.0,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "right.json"
            profile.save(path)
            loaded = UrdfZeroProfile.load(path)

        self.assertEqual(loaded, profile)
        self.assertEqual(loaded.pending_groups, (
            "thumb", "index", "middle", "ring", "pinky"
        ))

    def test_stream_mapping_must_match_capture_device(self) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))

        with self.assertRaisesRegex(ValueError, "enc_map"):
            profile.validate_stream(
                21,
                tuple(reversed(range(21))),
                zeroed=False,
                range_min=0.0,
                range_max=360.0,
            )

    def test_stream_coordinate_mode_must_match_capture(self) -> None:
        profile = UrdfZeroProfile.empty("right", tuple(range(21)))

        with self.assertRaisesRegex(ValueError, "zeroed.*不一致"):
            profile.validate_stream(
                21,
                tuple(range(21)),
                zeroed=True,
                range_min=-180.0,
                range_max=180.0,
            )


class UrdfZeroCalibrationCommandTest(unittest.TestCase):
    def test_command_captures_five_digits_then_pinky_cmc_and_saves(
        self,
    ) -> None:
        class StableConnection:
            channels = 21
            cs_by_joint = list(range(21))
            zeroed = False
            range_min = 0.0
            range_max = 360.0
            sequence = 0

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read_frame(self, **_kwargs):
                current = self.sequence
                self.sequence += 1
                return EncoderFrame(
                    sequence=current,
                    timestamp_ns=current * 10_000_000,
                    dropped=0,
                    angles_deg=[float(index) for index in range(1, 22)],
                )

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "right.json"
            args = SimpleNamespace(
                hand="right",
                host="unused",
                port=9100,
                timeout=5.0,
                output=str(output),
                sample_seconds=0.01,
                max_motion_deg=1.0,
            )
            with (
                patch(
                    "data_glove_wuji_teleop.application.encoder_capture."
                    "EncoderConnection.connect",
                    return_value=StableConnection(),
                ) as connect,
                patch("builtins.input", return_value=""),
                patch("sys.stdout"),
            ):
                result = command_calibrate_urdf_zero(args)

            profile = UrdfZeroProfile.load(output)

        self.assertEqual(result, 0)
        self.assertEqual(connect.call_count, 6)
        self.assertTrue(profile.complete)
        self.assertEqual(
            profile.captured_groups,
            tuple(name for name, _channels in EXPECTED_GROUP_CHANNELS),
        )

    def test_command_resumes_and_skips_already_saved_group(self) -> None:
        class StableConnection:
            channels = 21
            cs_by_joint = list(range(21))
            zeroed = False
            range_min = 0.0
            range_max = 360.0
            sequence = 0

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read_frame(self, **_kwargs):
                current = self.sequence
                self.sequence += 1
                return frame(current, [30.0] * 21)

        profile = UrdfZeroProfile.empty("right", tuple(range(21)))
        profile, _ = profile.capture_group(
            "thumb",
            [[10.0] * 21, [10.0] * 21],
            max_motion_deg=1.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "right.json"
            profile.save(output)
            args = SimpleNamespace(
                hand="right",
                host="unused",
                port=9100,
                timeout=5.0,
                output=str(output),
                sample_seconds=0.01,
                max_motion_deg=1.0,
            )
            with (
                patch(
                    "data_glove_wuji_teleop.application.encoder_capture."
                    "EncoderConnection.connect",
                    return_value=StableConnection(),
                ),
                patch("builtins.input", return_value=""),
                patch("sys.stdout"),
            ):
                command_calibrate_urdf_zero(args)
            loaded = UrdfZeroProfile.load(output)

        self.assertTrue(loaded.complete)
        expected_angles = [30.0] * 21
        for index in URDF_ZERO_GROUPS[0].channel_indices:
            expected_angles[index] = 10.0
        self.assertEqual(loaded.apply_angles(expected_angles), [0.0] * 21)


if __name__ == "__main__":
    unittest.main()
