"""离线数据手套回放的公开记录与时间线合同测试。"""

from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import h5py
import numpy as np

from data_glove_wuji_teleop.adapters.retargeting.glove_pipeline import (
    GloveRetargetPipeline,
)
from data_glove_wuji_teleop.replay import (
    GloveRecording,
    ReplayTimeline,
    discover_recordings,
    frame_index_at_timestamp,
    resolve_recording,
)


def _write_recording(
    root: Path,
    recording_id: str,
    *,
    start_timestamp_ns: int,
) -> Path:
    path = root / recording_id / "right_glove" / "dataglove.h5"
    path.parent.mkdir(parents=True)
    with h5py.File(path, "w") as output:
        output.attrs["schema"] = "dataglove_dataset"
        output.attrs["schema_version"] = 2
        output.attrs["dataset_id"] = f"{recording_id}/right_glove"
        metadata = output.create_group("metadata")
        metadata.attrs["complete"] = 1
        metadata.attrs["logical_slot"] = "right_hand"
        metadata.attrs["recording_uuid"] = recording_id
        metadata.attrs["start_timestamp_ns"] = start_timestamp_ns
        metadata.attrs["stop_timestamp_ns"] = start_timestamp_ns + 20_000_000
        metadata.create_dataset(
            "dataset.json",
            data=json.dumps(
                {
                    "sensors": {
                        "cameras": [
                            {
                                "id": "cam0",
                                "display_rotation_degrees_ccw": 90,
                            }
                        ]
                    }
                }
            ),
        )
        encoder = output.create_group("sensor/encoder")
        encoder.create_dataset(
            "timestamp_ns",
            data=np.array(
                [start_timestamp_ns, start_timestamp_ns + 10_000_000],
                dtype=np.int64,
            ),
        )
        encoder.create_dataset(
            "joint_deg",
            data=np.array(
                [
                    [1.25] + [0.0] * 20,
                    [2.5] + [0.0] * 20,
                ],
                dtype=np.float64,
            ),
        )
        camera = output.create_group("sensor/camera/cam0")
        camera.attrs["video_path"] = "video/cam.mp4"
        camera.create_dataset(
            "timestamp_ns",
            data=np.array(
                [start_timestamp_ns + 5_000_000],
                dtype=np.int64,
            ),
        )
        calibration = output.create_group("calib")
        calibration.create_dataset(
            "calibration.json",
            data=json.dumps(
                {
                    "encoder": {
                        "channels": 21,
                        "unit": "deg",
                        "zero": [10.0 + index for index in range(21)],
                        "joint_to_cs": list(reversed(range(21))),
                    }
                }
            ),
        )
    video = path.parent / "video" / "cam.mp4"
    video.parent.mkdir()
    video.write_bytes(b"fixture-video")
    return path


class RecordingCatalogTest(unittest.TestCase):

    def test_latest_resolves_to_complete_recording_with_newest_start(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            older = _write_recording(
                root,
                "00000000-0000-0000-0000-000000000001",
                start_timestamp_ns=1_000_000_000,
            )
            latest = _write_recording(
                root,
                "00000000-0000-0000-0000-000000000002",
                start_timestamp_ns=2_000_000_000,
            )
            unsupported = _write_recording(
                root,
                "00000000-0000-0000-0000-000000000099",
                start_timestamp_ns=3_000_000_000,
            )
            with h5py.File(unsupported, "r+") as output:
                output.attrs["schema_version"] = 99

            discovered = discover_recordings(root)

            self.assertEqual(
                [item.path for item in discovered],
                [older, latest],
            )
            self.assertEqual(resolve_recording(root, "latest"), latest)

    def test_open_rejects_unsupported_or_incomplete_hdf5_contract(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            unsupported = _write_recording(
                root,
                "00000000-0000-0000-0000-000000000005",
                start_timestamp_ns=5_000_000_000,
            )
            with h5py.File(unsupported, "r+") as output:
                output.attrs["schema_version"] = 99
            with self.assertRaisesRegex(ValueError, "schema_version"):
                GloveRecording.open(unsupported)

            incomplete = _write_recording(
                root,
                "00000000-0000-0000-0000-000000000006",
                start_timestamp_ns=6_000_000_000,
            )
            with h5py.File(incomplete, "r+") as output:
                del output["sensor/encoder/timestamp_ns"]
            with self.assertRaisesRegex(ValueError, "HDF5 结构不完整"):
                GloveRecording.open(incomplete)

    def test_open_exposes_encoder_calibration_and_synchronized_video(self) -> None:
        with TemporaryDirectory() as directory:
            path = _write_recording(
                Path(directory),
                "00000000-0000-0000-0000-000000000003",
                start_timestamp_ns=3_000_000_000,
            )

            recording = GloveRecording.open(path)

            self.assertEqual(recording.joint_deg.shape, (2, 21))
            self.assertEqual(recording.joint_deg[:, 0].tolist(), [1.25, 2.5])
            self.assertEqual(
                recording.encoder_timestamps_ns.tolist(),
                [3_000_000_000, 3_010_000_000],
            )
            self.assertEqual(recording.zero_offsets_deg[0], 10.0)
            self.assertEqual(recording.joint_to_cs, tuple(reversed(range(21))))
            self.assertEqual(
                recording.camera_timestamps_ns.tolist(),
                [3_005_000_000],
            )
            self.assertEqual(recording.video_path, path.parent / "video/cam.mp4")
            self.assertEqual(recording.video_rotation_degrees_ccw, 90)

            frame = recording.encoder_frame(0)
            zeroed = recording.embedded_zero_profile().apply_angles(
                frame.angles_deg
            )
            self.assertEqual(frame.sequence, 0)
            self.assertEqual(frame.timestamp_ns, 3_000_000_000)
            self.assertAlmostEqual(zeroed[0], -8.75)


class ReplayTimelineTest(unittest.TestCase):
    def test_speed_and_loop_select_encoder_and_video_from_same_timestamp(self) -> None:
        timeline = ReplayTimeline(
            np.array(
                [1_000_000_000, 1_010_000_000, 1_030_000_000],
                dtype=np.int64,
            ),
            speed=2.0,
            loop=True,
        )

        before_loop = timeline.position_at(0.006)
        last_frame = timeline.position_at(0.020)
        after_loop = timeline.position_at(0.025)

        self.assertEqual(before_loop.frame_index, 1)
        self.assertEqual(before_loop.recording_timestamp_ns, 1_012_000_000)
        self.assertEqual(before_loop.loop_count, 0)
        self.assertEqual(last_frame.frame_index, 2)
        self.assertEqual(after_loop.frame_index, 0)
        self.assertEqual(after_loop.recording_timestamp_ns, 1_005_000_000)
        self.assertEqual(after_loop.loop_count, 1)
        self.assertEqual(
            frame_index_at_timestamp(
                np.array(
                    [1_005_000_000, 1_015_000_000, 1_025_000_000],
                    dtype=np.int64,
                ),
                before_loop.recording_timestamp_ns,
            ),
            0,
        )


class ReplayFrameProcessingTest(unittest.TestCase):
    def test_recorded_frame_updates_glove_fk_and_exports_skeleton(self) -> None:
        project_root = Path(__file__).resolve().parents[2]
        with TemporaryDirectory() as directory:
            path = _write_recording(
                Path(directory),
                "00000000-0000-0000-0000-000000000004",
                start_timestamp_ns=4_000_000_000,
            )
            recording = GloveRecording.open(path)
            pipeline = GloveRetargetPipeline.load(
                glove_urdf=(
                    project_root
                    / "assets/data_glove_urdf/data_glove_urdf_R/urdf/data_glove_urdf_R.urdf"
                ),
                glove_config=(
                    project_root
                    / "config/dataglove/urdf/CH02260812AR001_local.json"
                ),
                zero_file=None,
                morphology_file=None,
                commissioning=True,
            )

            points = pipeline.process_frame(
                recording.encoder_frame(0),
                zero_profile=recording.embedded_zero_profile(),
            )

            self.assertEqual(points.shape, (21, 3))
            self.assertTrue(np.isfinite(points).all())


if __name__ == "__main__":
    unittest.main()
