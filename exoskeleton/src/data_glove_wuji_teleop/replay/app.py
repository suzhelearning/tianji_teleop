"""录制的数据手套、骨架、MANO 与 Wuji Hand2 同步回放。"""

from __future__ import annotations

import argparse
import math
import re
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from data_glove_wuji_teleop.adapters.retargeting.glove_pipeline import (
    GloveRetargetPipeline,
)
from data_glove_wuji_teleop.adapters.retargeting.wuji_official import (
    OfficialWujiRetargetAdapter,
    WujiRetargetResult,
    compute_tip_fit_metrics,
)
from data_glove_wuji_teleop.adapters.retargeting.wuji_process import launch_official_adapter
from data_glove_wuji_teleop.adapters.runtime.simulation_session import SimulationLease
from data_glove_wuji_teleop.adapters.simulation.mujoco_dataglove import DEFAULT_URDF
from data_glove_wuji_teleop.adapters.simulation.mujoco_retarget_composite import (
    load_retarget_composite_model,
)
from data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_official import (
    NamedQposBinding,
    _DEFAULT_WORKER_PYTHON,
    _DEFAULT_WUJI_CHECKOUT,
    apply_named_qpos,
    build_named_qpos_binding,
    update_mediapipe_target_scene,
)
from data_glove_wuji_teleop.profiles.dataglove.urdf_zero import UrdfZeroProfile
from data_glove_wuji_teleop.profiles.wuji_v2.retargeting import DEFAULT_MANO_MORPHOLOGY
from data_glove_wuji_teleop.project import PROJECT_ROOT, get_hand_resources

from .recording import (
    GloveRecording,
    discover_recordings,
    resolve_recording,
)
from .timeline import ReplayTimeline
from .video import SynchronizedVideoPlayer


DEFAULT_RECORDS_ROOT = PROJECT_ROOT / "data/records"
DEFAULT_GLOVE_ID = "CH02260812AR001"
_FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


@dataclass
class ReplayRenderer:
    """将一帧录制同时写入手套 URDF 和 Wuji Hand2。"""

    glove: GloveRetargetPipeline
    zero_profile: UrdfZeroProfile | None
    retargeter: OfficialWujiRetargetAdapter
    model: mujoco.MjModel
    data: mujoco.MjData
    glove_binding: NamedQposBinding
    glove_joint_names: tuple[str, ...]
    glove_display_rotation: np.ndarray
    column_spacing: float
    apply_filter: bool
    wuji_binding: NamedQposBinding | None = None

    @classmethod
    def create(
        cls,
        *,
        glove: GloveRetargetPipeline,
        zero_profile: UrdfZeroProfile | None,
        retargeter: OfficialWujiRetargetAdapter,
        glove_urdf: Path,
        column_spacing: float,
        apply_filter: bool,
    ) -> "ReplayRenderer":
        resources = get_hand_resources("right", generation="v2")
        if resources.mjcf is None:
            raise ValueError("Wuji Hand2 右手 MJCF 未配置")
        model, data = load_retarget_composite_model(
            glove_urdf=glove_urdf,
            wuji_mjcf=resources.mjcf,
            column_spacing=column_spacing,
        )
        mujoco.mj_forward(model, data)
        wrist_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "glove_wrist_link",
        )
        if wrist_id < 0:
            raise ValueError("回放组合模型缺少 glove_wrist_link")
        joint_names = tuple(glove.qpos_by_joint)
        return cls(
            glove=glove,
            zero_profile=zero_profile,
            retargeter=retargeter,
            model=model,
            data=data,
            glove_binding=build_named_qpos_binding(
                model,
                joint_names,
                model_prefix="glove_",
                require_complete_model=False,
            ),
            glove_joint_names=joint_names,
            glove_display_rotation=data.xmat[wrist_id].reshape(3, 3).copy(),
            column_spacing=float(column_spacing),
            apply_filter=bool(apply_filter),
        )

    def apply_recording_frame(
        self,
        recording: GloveRecording,
        frame_index: int,
    ) -> WujiRetargetResult:
        keypoints = self.glove.process_frame(
            recording.encoder_frame(frame_index),
            zero_profile=self.zero_profile,
        )
        result = self.retargeter.retarget(
            keypoints,
            apply_filter=self.apply_filter,
        )
        if self.wuji_binding is None:
            self.wuji_binding = build_named_qpos_binding(
                self.model,
                result.joint_names,
                model_prefix="wuji_",
                require_complete_model=False,
            )
        elif result.joint_names != self.wuji_binding.joint_names:
            raise RuntimeError("官方 worker 在回放中改变了关节顺序")
        glove_values = np.array(
            [
                self.glove.data.qpos[self.glove.qpos_by_joint[name]]
                for name in self.glove_joint_names
            ],
            dtype=np.float64,
        )
        apply_named_qpos(self.data, self.glove_binding, glove_values)
        apply_named_qpos(self.data, self.wuji_binding, result.qpos)
        mujoco.mj_forward(self.model, self.data)
        return result

    def update_overlays(
        self,
        scene: mujoco.MjvScene,
        result: WujiRetargetResult,
    ) -> None:
        offset = np.array(
            (-self.column_spacing, 0.0, 0.0),
            dtype=np.float64,
        )
        rotated_data = _rotated_skeleton_data(
            self.glove.data,
            self.glove_display_rotation,
        )
        self.glove.landmarks.skeleton.update_scene(
            rotated_data,
            scene,
            world_offset=offset,
            clear_scene=True,
        )
        update_mediapipe_target_scene(
            self.model,
            self.data,
            scene,
            result.transformed_keypoints,
            world_offset=offset,
            wrist_body_name="wuji_r_wrist",
            clear_scene=False,
        )


@dataclass(frozen=True)
class _SkeletonPose:
    """手套骨架在组合场景显示坐标中的位姿数组。"""

    xanchor: np.ndarray
    xpos: np.ndarray
    xmat: np.ndarray


def _rotated_skeleton_data(
    data: mujoco.MjData,
    rotation: np.ndarray,
) -> _SkeletonPose:
    matrices = np.asarray(data.xmat, dtype=np.float64).reshape(-1, 3, 3)
    return _SkeletonPose(
        xanchor=np.asarray(data.xanchor, dtype=np.float64) @ rotation.T,
        xpos=np.asarray(data.xpos, dtype=np.float64) @ rotation.T,
        xmat=(rotation[None, :, :] @ matrices).reshape(-1, 9),
    )


def _select_zero_profile(
    args: argparse.Namespace,
    recording: GloveRecording,
) -> UrdfZeroProfile | None:
    if args.zero_source == "recorded":
        return recording.embedded_zero_profile()
    if args.zero_source == "none":
        return None
    path = args.zero_file or (
        PROJECT_ROOT
        / "config/dataglove/urdf/zero"
        / f"{args.glove_id}_local.json"
    )
    return UrdfZeroProfile.load(path)


def _validate_recording_contract(
    recording: GloveRecording,
    glove: GloveRetargetPipeline,
    zero_profile: UrdfZeroProfile | None,
) -> None:
    glove.mapping.validate_stream(
        channels=21,
        cs_by_joint=recording.joint_to_cs,
    )
    if zero_profile is not None:
        zero_profile.validate_stream(
            21,
            recording.joint_to_cs,
            zeroed=False,
            range_min=0.0,
            range_max=360.0,
        )


def _video_context(
    args: argparse.Namespace,
    recording: GloveRecording,
) -> SynchronizedVideoPlayer | nullcontext[None]:
    if args.no_video:
        return nullcontext()
    if recording.video_path is None or recording.camera_timestamps_ns.size == 0:
        print("[视频] 录制不包含可用 cam0 MP4，仅回放 MuJoCo")
        return nullcontext()
    rotation = (
        recording.video_rotation_degrees_ccw
        if args.video_rotation == "auto"
        else int(args.video_rotation)
    )
    return SynchronizedVideoPlayer(
        recording.video_path,
        recording.camera_timestamps_ns,
        rotation_degrees_ccw=rotation,
        max_long_edge=args.video_size,
        window_name=f"{args.glove_id} camera replay",
        window_x=args.video_window_x,
        window_y=args.video_window_y,
    )


def _run_headless(
    args: argparse.Namespace,
    recording: GloveRecording,
    renderer: ReplayRenderer,
) -> None:
    requested = args.max_frames if args.max_frames > 0 else 1
    count = min(requested, recording.frame_count)
    indices = np.linspace(0, recording.frame_count - 1, count, dtype=int)
    scene = mujoco.MjvScene(renderer.model, maxgeom=128)
    result: WujiRetargetResult | None = None
    for index in indices:
        result = renderer.apply_recording_frame(recording, int(index))
        renderer.update_overlays(scene, result)
    if result is None:
        raise RuntimeError("headless 回放没有处理任何帧")
    print(
        f"[headless] 已处理 {count} 帧，"
        f"MuJoCo nq={renderer.model.nq}，"
        f"overlay_geoms={scene.ngeom}，"
        f"last_cost={result.cost:.4f}",
        flush=True,
    )


def _run_viewer(
    args: argparse.Namespace,
    recording: GloveRecording,
    renderer: ReplayRenderer,
) -> None:
    timeline = ReplayTimeline(
        recording.encoder_timestamps_ns,
        speed=args.speed,
        loop=args.loop,
    )
    with (
        SimulationLease(generation="v2", hand="right"),
        _video_context(args, recording) as video,
        mujoco.viewer.launch_passive(
            renderer.model,
            renderer.data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer,
    ):
        viewer.cam.lookat[:] = renderer.data.xpos[1:].mean(axis=0)
        viewer.cam.distance = 0.85
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -20.0
        started = time.monotonic()
        last_index: int | None = None
        last_loop = -1
        processed = 0
        last_report = 0.0
        while viewer.is_running():
            now = time.monotonic()
            position = timeline.position_at(now - started)
            changed = (
                position.frame_index != last_index
                or position.loop_count != last_loop
            )
            if changed:
                result = renderer.apply_recording_frame(
                    recording,
                    position.frame_index,
                )
                renderer.update_overlays(viewer.user_scn, result)
                last_index = position.frame_index
                last_loop = position.loop_count
                processed += 1
                if now - last_report >= args.print_interval:
                    metrics = compute_tip_fit_metrics(result)
                    errors = ", ".join(
                        f"{name}={value * 1000:.1f}mm"
                        for name, value in zip(
                            _FINGER_NAMES,
                            metrics.position_error_m,
                        )
                    )
                    print(
                        f"[replay] loop={position.loop_count} "
                        f"frame={position.frame_index + 1}/"
                        f"{recording.frame_count} cost={result.cost:.4f} "
                        f"tip={errors}",
                        flush=True,
                    )
                    last_report = now
            if video is not None and not video.show_at(
                position.recording_timestamp_ns,
                loop_count=position.loop_count,
            ):
                break
            viewer.sync()
            if position.finished:
                break
            if args.max_frames > 0 and processed >= args.max_frames:
                break
            if not changed:
                time.sleep(0.001)


def _print_recordings(records_root: Path) -> None:
    recordings = discover_recordings(records_root)
    if not recordings:
        print(f"{records_root} 中没有完整的右手套录制")
        return
    for item in recordings:
        print(
            f"{item.recording_id}  frames={item.encoder_frames:<6d} "
            f"duration={item.duration_seconds:7.2f}s  {item.path}"
        )


def run(args: argparse.Namespace) -> int:
    if not re.fullmatch(r"[A-Za-z0-9._-]+", args.glove_id):
        raise ValueError("--glove-id 只能包含字母、数字、点、下划线和连字符")
    if args.list:
        _print_recordings(args.records_dir)
        return 0
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise ValueError("--speed 必须是正有限数值")
    if args.max_frames < 0:
        raise ValueError("--max-frames 不能为负")
    if args.video_size <= 0:
        raise ValueError("--video-size 必须为正")
    if not math.isfinite(args.worker_timeout) or args.worker_timeout <= 0.0:
        raise ValueError("--worker-timeout 必须是正有限数值")

    recording_path = resolve_recording(args.records_dir, args.recording)
    recording = GloveRecording.open(recording_path)
    glove_config = args.glove_config or (
        PROJECT_ROOT
        / "config/dataglove/urdf"
        / f"{args.glove_id}_local.json"
    )
    print(
        f"[录制] {recording.recording_id}  "
        f"frames={recording.frame_count}  "
        f"duration={recording.duration_seconds:.2f}s",
        flush=True,
    )
    print(f"[手套] id={args.glove_id}  config={glove_config}", flush=True)
    zero_profile = _select_zero_profile(args, recording)
    print(f"[零位] source={args.zero_source}", flush=True)
    glove = GloveRetargetPipeline.load(
        glove_urdf=args.glove_urdf,
        glove_config=glove_config,
        zero_file=None,
        morphology_file=args.mano_morphology,
        commissioning=args.commission_directions,
    )
    _validate_recording_contract(recording, glove, zero_profile)

    print("[Wuji] 启动官方 Hand2 retarget worker...", flush=True)
    with launch_official_adapter(
        args,
        response_timeout=args.worker_timeout,
        start_new_session=True,
    ) as retargeter:
        renderer = ReplayRenderer.create(
            glove=glove,
            zero_profile=zero_profile,
            retargeter=retargeter,
            glove_urdf=args.glove_urdf,
            column_spacing=args.column_spacing,
            apply_filter=args.apply_filter,
        )
        if args.headless:
            _run_headless(args, recording, renderer)
        else:
            print(
                "[MuJoCo] 三列回放：手套 URDF+彩色骨架 / "
                "黄色 MANO / Wuji Hand2",
                flush=True,
            )
            _run_viewer(args, recording, renderer)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "回放 HDF5 数据手套录制，同步显示视频、"
            "手套 URDF/骨架/MANO 与 Wuji Hand2"
        )
    )
    parser.set_defaults(hand="right", pinch_d1_cm=None, pinch_alpha_max=1.0)
    parser.add_argument("--records-dir", type=Path, default=DEFAULT_RECORDS_ROOT)
    parser.add_argument(
        "--recording",
        default="latest",
        help="latest、录制 UUID、会话目录或 dataglove.h5 路径",
    )
    parser.add_argument("--list", action="store_true", help="列出可回放录制")
    parser.add_argument("--glove-id", default=DEFAULT_GLOVE_ID)
    parser.add_argument("--glove-urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--glove-config", type=Path, default=None)
    parser.add_argument(
        "--zero-source",
        choices=("recorded", "device", "none"),
        default="device",
        help=(
            "device=按 glove-id 使用当前手套六步零位（默认），"
            "recorded=录制嵌入零位，none=不扣零位"
        ),
    )
    parser.add_argument("--zero-file", type=Path, default=None)
    parser.add_argument("--mano-morphology", type=Path, default=DEFAULT_MANO_MORPHOLOGY)
    parser.add_argument("--commission-directions", action="store_true")
    parser.add_argument("--worker-python", type=Path, default=_DEFAULT_WORKER_PYTHON)
    parser.add_argument(
        "--worker-timeout",
        type=float,
        default=1.0,
        help="每帧官方 retarget 的最长等待秒数；初始化单独等待 30 秒",
    )
    parser.add_argument("--wuji-checkout", type=Path, default=_DEFAULT_WUJI_CHECKOUT)
    parser.add_argument("--wuji-config", type=Path, default=None)
    parser.add_argument("--wuji-urdf", type=Path, default=None)
    parser.add_argument("--apply-filter", action="store_true")
    parser.add_argument("--column-spacing", type=float, default=0.22)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--video-size", type=int, default=720)
    parser.add_argument(
        "--video-rotation",
        choices=("auto", "0", "90", "180", "270"),
        default="0",
        help="视频逆时针旋转角度；默认 0 为横屏，auto 读录制元数据",
    )
    parser.add_argument("--video-window-x", type=int, default=1400)
    parser.add_argument("--video-window-y", type=int, default=100)
    parser.add_argument("--print-interval", type=float, default=1.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="0=不限；headless 模式下 0 表示仅验证 1 帧",
    )
    return parser


def main() -> None:
    try:
        raise SystemExit(run(build_parser().parse_args()))
    except KeyboardInterrupt:
        print("\n回放已停止。", flush=True)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        raise SystemExit(f"错误：{exc}") from exc
