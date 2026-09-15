"""HDF5 数据手套录制的发现与选择。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

from data_glove_wuji_teleop.adapters.glove.encoder_stream import EncoderFrame
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import (
    URDF_ZERO_GROUPS,
    UrdfZeroProfile,
)


@dataclass(frozen=True)
class RecordingSummary:
    """一个可回放的右手套录制。"""

    path: Path
    recording_id: str
    start_timestamp_ns: int
    stop_timestamp_ns: int
    encoder_frames: int

    @property
    def duration_seconds(self) -> float:
        return (self.stop_timestamp_ns - self.start_timestamp_ns) / 1e9


@dataclass(frozen=True)
class GloveRecording:
    """已验证并载入内存的 21 路手套录制。"""

    path: Path
    recording_id: str
    encoder_timestamps_ns: np.ndarray
    joint_deg: np.ndarray
    zero_offsets_deg: tuple[float, ...]
    joint_to_cs: tuple[int, ...]
    camera_timestamps_ns: np.ndarray
    video_path: Path | None
    video_rotation_degrees_ccw: int

    @classmethod
    def open(cls, filepath: str | Path) -> "GloveRecording":
        path = Path(filepath).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"录制 HDF5 不存在：{path}")
        try:
            return cls._open_hdf5(path)
        except KeyError as exc:
            raise ValueError(
                f"录制 HDF5 结构不完整：缺少 {exc}"
            ) from exc
        except OSError as exc:
            raise ValueError(f"录制 HDF5 无法读取：{path}") from exc

    @classmethod
    def _open_hdf5(cls, path: Path) -> "GloveRecording":
        with h5py.File(path, "r") as source:
            if _text(source.attrs.get("schema", "")) != "dataglove_dataset":
                raise ValueError("录制 schema 必须是 dataglove_dataset")
            schema_version = source.attrs.get("schema_version")
            if type(schema_version) not in (int, np.int32, np.int64) or int(
                schema_version
            ) != 2:
                raise ValueError("录制 schema_version 必须是 2")
            metadata = source["metadata"]
            if not bool(metadata.attrs.get("complete", False)):
                raise ValueError("录制尚未完整收尾")
            if _text(metadata.attrs.get("logical_slot", "")) != "right_hand":
                raise ValueError("当前回放只支持 right_hand 录制")
            timestamps = np.asarray(
                source["sensor/encoder/timestamp_ns"],
                dtype=np.int64,
            )
            joints = np.asarray(
                source["sensor/encoder/joint_deg"],
                dtype=np.float64,
            )
            _validate_encoder_arrays(timestamps, joints)
            calibration = _load_json_scalar(
                source["calib/calibration.json"],
                label="嵌入式标定",
            )
            encoder = calibration.get("encoder")
            if not isinstance(encoder, dict):
                raise ValueError("嵌入式标定缺少 encoder")
            if encoder.get("channels") != 21:
                raise ValueError("encoder.channels 必须是 21")
            if encoder.get("unit") != "deg":
                raise ValueError("encoder.unit 必须是 deg")
            zero_offsets = _float_tuple(encoder.get("zero"), "encoder.zero")
            joint_to_cs = _int_tuple(
                encoder.get("joint_to_cs"),
                "encoder.joint_to_cs",
            )
            if len(zero_offsets) != 21:
                raise ValueError("encoder.zero 必须包含 21 路")
            if tuple(sorted(joint_to_cs)) != tuple(range(21)):
                raise ValueError("encoder.joint_to_cs 必须是 0..20 排列")
            camera_timestamps, video_path, video_rotation = _load_camera(
                source,
                path.parent,
            )
            return cls(
                path=path,
                recording_id=_text(metadata.attrs["recording_uuid"]),
                encoder_timestamps_ns=timestamps,
                joint_deg=joints,
                zero_offsets_deg=zero_offsets,
                joint_to_cs=joint_to_cs,
                camera_timestamps_ns=camera_timestamps,
                video_path=video_path,
                video_rotation_degrees_ccw=video_rotation,
            )

    @property
    def frame_count(self) -> int:
        return int(self.joint_deg.shape[0])

    @property
    def duration_seconds(self) -> float:
        return float(
            self.encoder_timestamps_ns[-1] - self.encoder_timestamps_ns[0]
        ) / 1e9

    def encoder_frame(self, index: int) -> EncoderFrame:
        """按原始序号返回一帧与实时协议同形的编码器数据。"""

        if type(index) is not int or not 0 <= index < self.frame_count:
            raise IndexError(f"回放帧序号越界：{index}")
        return EncoderFrame(
            sequence=index,
            timestamp_ns=int(self.encoder_timestamps_ns[index]),
            dropped=0,
            angles_deg=self.joint_deg[index].tolist(),
        )

    def embedded_zero_profile(self) -> UrdfZeroProfile:
        """将录制当时的 encoder.zero 转为 URDF 软件零位。"""

        return UrdfZeroProfile(
            hand="right",
            expected_cs_by_joint=self.joint_to_cs,
            stream_zeroed=False,
            stream_range_deg=(0.0, 360.0),
            zero_offsets_deg=self.zero_offsets_deg,
            captured_groups=tuple(group.name for group in URDF_ZERO_GROUPS),
        )


def _read_summary(path: Path) -> RecordingSummary | None:
    try:
        with h5py.File(path, "r") as source:
            if _text(source.attrs.get("schema", "")) != "dataglove_dataset":
                return None
            if int(source.attrs.get("schema_version", -1)) != 2:
                return None
            metadata = source["metadata"]
            if not bool(metadata.attrs.get("complete", False)):
                return None
            if _text(metadata.attrs.get("logical_slot", "")) != "right_hand":
                return None
            timestamps = source["sensor/encoder/timestamp_ns"]
            joints = source["sensor/encoder/joint_deg"]
            if joints.ndim != 2 or joints.shape[1] != 21:
                return None
            if timestamps.shape != (joints.shape[0],) or joints.shape[0] == 0:
                return None
            calibration = _load_json_scalar(
                source["calib/calibration.json"],
                label="嵌入式标定",
            )
            encoder = calibration.get("encoder")
            if (
                not isinstance(encoder, dict)
                or encoder.get("channels") != 21
                or encoder.get("unit") != "deg"
            ):
                return None
            return RecordingSummary(
                path=path.resolve(),
                recording_id=_text(metadata.attrs["recording_uuid"]),
                start_timestamp_ns=int(timestamps[0]),
                stop_timestamp_ns=int(timestamps[-1]),
                encoder_frames=int(joints.shape[0]),
            )
    except (KeyError, OSError, TypeError, ValueError):
        return None


def _text(value: object) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def _load_json_scalar(dataset: h5py.Dataset, *, label: str) -> dict[str, object]:
    try:
        value = json.loads(_text(dataset[()]))
    except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} 不是有效 JSON") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} 必须是 JSON 对象")
    return value


def _float_tuple(value: object, label: str) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{label} 必须是数组")
    converted = tuple(float(item) for item in value)
    if not np.isfinite(converted).all():
        raise ValueError(f"{label} 包含非有限数值")
    return converted


def _int_tuple(value: object, label: str) -> tuple[int, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{label} 必须是数组")
    if any(type(item) is not int for item in value):
        raise ValueError(f"{label} 必须只包含整数")
    return tuple(value)


def _validate_encoder_arrays(
    timestamps: np.ndarray,
    joints: np.ndarray,
) -> None:
    if joints.ndim != 2 or joints.shape[1] != 21 or joints.shape[0] == 0:
        raise ValueError("joint_deg 必须是非空 (N, 21) 数组")
    if timestamps.shape != (joints.shape[0],):
        raise ValueError("encoder 时间戳数量与关节帧不一致")
    if not np.isfinite(joints).all():
        raise ValueError("joint_deg 包含非有限数值")
    if timestamps.size > 1 and np.any(np.diff(timestamps) <= 0):
        raise ValueError("encoder 时间戳必须严格递增")


def _load_camera(
    source: h5py.File,
    recording_directory: Path,
) -> tuple[np.ndarray, Path | None, int]:
    if "sensor/camera/cam0" not in source:
        return np.empty(0, dtype=np.int64), None, 0
    camera = source["sensor/camera/cam0"]
    timestamps = np.asarray(camera["timestamp_ns"], dtype=np.int64)
    if timestamps.ndim != 1:
        raise ValueError("camera 时间戳必须是一维数组")
    if timestamps.size > 1 and np.any(np.diff(timestamps) <= 0):
        raise ValueError("camera 时间戳必须严格递增")
    relative = _text(camera.attrs.get("video_path", ""))
    if not relative:
        return timestamps, None, _load_video_rotation(source)
    video = (recording_directory / relative).resolve()
    if recording_directory.resolve() not in video.parents:
        raise ValueError("camera video_path 超出录制目录")
    return (
        timestamps,
        video if video.is_file() else None,
        _load_video_rotation(source),
    )


def _load_video_rotation(source: h5py.File) -> int:
    if "metadata/dataset.json" not in source:
        return 0
    manifest = _load_json_scalar(
        source["metadata/dataset.json"],
        label="录制 manifest",
    )
    sensors = manifest.get("sensors")
    cameras = sensors.get("cameras") if isinstance(sensors, dict) else None
    if not isinstance(cameras, list):
        return 0
    for camera in cameras:
        if not isinstance(camera, dict) or camera.get("id") != "cam0":
            continue
        rotation = camera.get("display_rotation_degrees_ccw", 0)
        if type(rotation) is not int or rotation not in (0, 90, 180, 270):
            raise ValueError("cam0 显示旋转必须是 0/90/180/270")
        return rotation
    return 0


def discover_recordings(records_root: str | Path) -> tuple[RecordingSummary, ...]:
    """按开始时间升序列出完整的右手套录制。"""

    root = Path(records_root).expanduser()
    if not root.is_dir():
        raise FileNotFoundError(f"录制目录不存在：{root}")
    summaries = tuple(
        summary
        for path in root.glob("*/right_glove/dataglove.h5")
        if (summary := _read_summary(path)) is not None
    )
    return tuple(
        sorted(
            summaries,
            key=lambda item: (
                item.path.stat().st_mtime_ns,
                item.start_timestamp_ns,
                item.recording_id,
            ),
        )
    )


def resolve_recording(records_root: str | Path, selector: str | Path) -> Path:
    """将 ``latest``、录制 UUID 或显式路径解析为 HDF5。"""

    root = Path(records_root).expanduser()
    selected = str(selector)
    recordings = discover_recordings(root)
    if selected == "latest":
        if not recordings:
            raise FileNotFoundError(f"{root} 中没有完整的右手套录制")
        return recordings[-1].path
    for recording in recordings:
        if recording.recording_id == selected:
            return recording.path
    explicit = Path(selected).expanduser()
    candidates = (
        explicit,
        explicit / "dataglove.h5",
        explicit / "right_glove" / "dataglove.h5",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"找不到录制：{selector}")
