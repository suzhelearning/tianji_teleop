#!/usr/bin/env python3
"""数据手套到 Wuji 系列语义 20-DOF 目标的交互标定流程。"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from ..adapters.glove.encoder_stream import EncoderFrame
from ..profiles.dataglove.mapping import (
    CHANNEL_COUNT,
    CalibrationProfile,
    build_profile_from_captures,
    shortest_angular_delta_deg,
)
from ..project import get_hand_resources
from .encoder_capture import CaptureWindow, read_encoder_window


THUMB_LATERAL_INDICES = (0,)
FINGER_LATERAL_INDICES = (1, 2, 3, 4)
THUMB_FLEX_INDICES = (5, 10, 15)
FINGER_FLEX_INDICES = (6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19)
DIGIT_NAMES = ("大拇指", "食指", "中指", "无名指", "小指")

DEFAULT_DISABLED_CHANNELS = ("J3", "J4", "J10", "J15", "J20")
DEFAULT_FINGER_FLEX_TARGET_DEG = (80.0, 80.0, 60.0)
DEFAULT_THUMB_FLEX_TARGET_DEG = {
    "right": (60.0, -36.0, -80.0),
    "left": (60.0, 36.0, 80.0),
}
DEFAULT_LATERAL_TARGET_DEG = 10.0
MAX_LATERAL_TARGET_DEG = 20.0
THUMB_LATERAL_NEGATIVE_TARGET_DEG = 7.5
DEFAULT_INVERTED_CHANNELS = {
    "right": (),
    "left": ("J1", "J2", "J3", "J4", "J5"),
}
DEFAULT_PORT = 5580


def _validated_values(values: Sequence[float], label: str) -> tuple[float, ...]:
    if len(values) != CHANNEL_COUNT:
        raise ValueError(f"{label} 必须包含 {CHANNEL_COUNT} 路")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} 包含非有限数值")
    return converted


@dataclass(frozen=True)
class StableCapture:
    """稳定姿态各通道的代表值及中央 80% 运动范围。"""

    values_deg: tuple[float, ...]
    motion_deg: tuple[float, ...]

    @classmethod
    def from_values(
        cls,
        values_deg: Sequence[float],
        motion_deg: Sequence[float] | None = None,
    ) -> "StableCapture":
        values = _validated_values(values_deg, "稳定姿态")
        motion = _validated_values(
            motion_deg if motion_deg is not None else [0.0] * CHANNEL_COUNT,
            "稳定姿态运动范围",
        )
        if any(value < 0.0 for value in motion):
            raise ValueError("稳定姿态运动范围不能为负")
        return cls(values_deg=values, motion_deg=motion)

    def relative_to(self, zero_offsets_deg: Sequence[float]) -> "StableCapture":
        offsets = _validated_values(
            zero_offsets_deg,
            "稳定姿态零位",
        )
        return StableCapture.from_values(
            [
                shortest_angular_delta_deg(value, offset)
                for value, offset in zip(self.values_deg, offsets)
            ],
            self.motion_deg,
        )


@dataclass(frozen=True)
class SweepCapture:
    """动态扫角窗口内各通道的最小值和最大值。"""

    minimum_deg: tuple[float, ...]
    maximum_deg: tuple[float, ...]

    @classmethod
    def from_ranges(
        cls,
        minimum_deg: Sequence[float],
        maximum_deg: Sequence[float],
    ) -> "SweepCapture":
        minimum = _validated_values(minimum_deg, "扫角最小值")
        maximum = _validated_values(maximum_deg, "扫角最大值")
        if any(low > high for low, high in zip(minimum, maximum)):
            raise ValueError("扫角最小值不能大于最大值")
        return cls(minimum_deg=minimum, maximum_deg=maximum)

    def relative_to(self, zero_offsets_deg: Sequence[float]) -> "SweepCapture":
        offsets = _validated_values(
            zero_offsets_deg,
            "扫角零位",
        )
        relative_pairs = [
            sorted(
                (
                    shortest_angular_delta_deg(low, offset),
                    shortest_angular_delta_deg(high, offset),
                )
            )
            for low, high, offset in zip(
                self.minimum_deg,
                self.maximum_deg,
                offsets,
            )
        ]
        return SweepCapture.from_ranges(
            [pair[0] for pair in relative_pairs],
            [pair[1] for pair in relative_pairs],
        )


def _frame_columns(frames: Iterable[EncoderFrame]) -> list[tuple[float, ...]]:
    rows = [
        _validated_values(frame.angles_deg, "编码器帧")
        for frame in frames
    ]
    if not rows:
        raise ValueError("采样窗口没有编码器帧")
    return list(zip(*rows))


def summarize_stable_frames(frames: Iterable[EncoderFrame]) -> StableCapture:
    """用中位数表示姿态，以 P90-P10 抵抗少量过渡或离群帧。"""

    columns = _frame_columns(frames)
    motion: list[float] = []
    for column in columns:
        if len(column) < 2:
            motion.append(0.0)
            continue
        deciles = statistics.quantiles(column, n=10, method="inclusive")
        motion.append(deciles[-1] - deciles[0])
    return StableCapture.from_values(
        [statistics.median(column) for column in columns],
        motion,
    )


def summarize_sweep_frames(frames: Iterable[EncoderFrame]) -> SweepCapture:
    """提取动态扫角窗口内各通道的最小值和最大值。"""

    columns = _frame_columns(frames)
    return SweepCapture.from_ranges(
        [min(column) for column in columns],
        [max(column) for column in columns],
    )


def validate_flex_capture(
    capture: StableCapture,
    indices: Sequence[int],
    *,
    disabled_channels: Iterable[str],
    label: str,
    max_motion_deg: float,
    max_motion_ratio: float,
    min_travel_deg: float,
) -> None:
    """验收稳定屈伸端点；大行程关节按相对比例放宽微小手部运动。"""

    disabled = {str(channel).upper() for channel in disabled_channels}
    motion_floor = float(max_motion_deg)
    motion_ratio = float(max_motion_ratio)
    travel_minimum = float(min_travel_deg)
    if not math.isfinite(motion_floor) or motion_floor < 0.0:
        raise ValueError("max_stable_motion_deg 必须是非负有限数值")
    if not math.isfinite(motion_ratio) or motion_ratio < 0.0:
        raise ValueError("max_stable_motion_ratio 必须是非负有限数值")
    if not math.isfinite(travel_minimum) or travel_minimum <= 0.0:
        raise ValueError("min_flex_travel_deg 必须是正有限数值")

    for glove_index in indices:
        joint = f"J{glove_index + 1}"
        if joint in disabled:
            continue
        endpoint = capture.values_deg[glove_index]
        allowed_motion = max(motion_floor, abs(endpoint) * motion_ratio)
        motion = capture.motion_deg[glove_index]
        if motion > allowed_motion:
            raise ValueError(
                f"{joint} {label}姿态不稳定: 中央80%运动 "
                f"{motion:.3f}° > 允许 {allowed_motion:.3f}°"
            )
        if abs(endpoint) < travel_minimum:
            raise ValueError(
                f"{joint} {label}幅度不足: {endpoint:+.3f}°"
            )


def validate_lateral_capture(
    capture: SweepCapture,
    indices: Sequence[int],
    *,
    disabled_channels: Iterable[str],
    min_travel_deg: float,
) -> None:
    """验收侧摆扫角；允许零位到单侧极限，也允许完整双侧扫角。"""

    disabled = {str(channel).upper() for channel in disabled_channels}
    travel_minimum = float(min_travel_deg)
    if not math.isfinite(travel_minimum) or travel_minimum <= 0.0:
        raise ValueError("min_lateral_travel_deg 必须是正有限数值")

    for glove_index in indices:
        joint = f"J{glove_index + 1}"
        if joint in disabled:
            continue
        low = capture.minimum_deg[glove_index]
        high = capture.maximum_deg[glove_index]
        negative_travel = max(0.0, -low)
        positive_travel = max(0.0, high)
        if max(negative_travel, positive_travel) < travel_minimum:
            raise ValueError(
                f"{joint} 侧摆有效量程不足: "
                f"[{low:+.3f}°, {high:+.3f}°]，"
                f"至少一个方向需达到 {travel_minimum:.3f}°"
            )


def build_workflow_profile(
    *,
    hand: str,
    cs_by_joint: Iterable[int],
    finger_flex: StableCapture,
    thumb_flex: StableCapture,
    finger_lateral: SweepCapture,
    thumb_lateral: SweepCapture,
    disabled_channels: Iterable[str] = DEFAULT_DISABLED_CHANNELS,
    deadband_deg: float = 0.0,
    finger_flex_target_deg: Sequence[float] = DEFAULT_FINGER_FLEX_TARGET_DEG,
    thumb_flex_target_deg: Sequence[float] | None = None,
    lateral_target_deg: float = DEFAULT_LATERAL_TARGET_DEG,
    inverted_channels: Iterable[str] = (),
    max_stable_motion_deg: float = 3.0,
    max_stable_motion_ratio: float = 0.1,
    min_flex_travel_deg: float = 5.0,
    min_lateral_travel_deg: float = 3.0,
    zero_offsets_deg: Sequence[float] = (0.0,) * CHANNEL_COUNT,
) -> CalibrationProfile:
    """将“大拇指独立”的四组采样结果组装为标定配置。"""

    if hand not in ("left", "right"):
        raise ValueError("hand 必须是 left 或 right")
    zero_offsets = _validated_values(
        zero_offsets_deg,
        "标定零位",
    )
    finger_flex = finger_flex.relative_to(zero_offsets)
    thumb_flex = thumb_flex.relative_to(zero_offsets)
    finger_lateral = finger_lateral.relative_to(zero_offsets)
    thumb_lateral = thumb_lateral.relative_to(zero_offsets)
    finger_targets = tuple(float(value) for value in finger_flex_target_deg)
    if len(finger_targets) != 3:
        raise ValueError("finger_flex_target_deg 必须包含三段目标")
    thumb_targets = tuple(
        float(value)
        for value in (
            thumb_flex_target_deg
            if thumb_flex_target_deg is not None
            else DEFAULT_THUMB_FLEX_TARGET_DEG[hand]
        )
    )
    if len(thumb_targets) != 3:
        raise ValueError("thumb_flex_target_deg 必须包含三段目标")
    lateral_target = float(lateral_target_deg)
    if not math.isfinite(lateral_target) or lateral_target <= 0.0:
        raise ValueError("lateral_target_deg 必须是正有限数值")
    if lateral_target > MAX_LATERAL_TARGET_DEG:
        raise ValueError(
            f"侧摆目标不能超过 Wuji 安全范围 "
            f"{MAX_LATERAL_TARGET_DEG:.1f}°"
        )

    disabled = {str(channel).upper() for channel in disabled_channels}
    inverted = {str(channel).upper() for channel in inverted_channels}
    lateral_minimum = float(min_lateral_travel_deg)
    if not math.isfinite(lateral_minimum) or lateral_minimum <= 0.0:
        raise ValueError("min_lateral_travel_deg 必须是正有限数值")

    validate_flex_capture(
        finger_flex,
        FINGER_FLEX_INDICES,
        disabled_channels=disabled,
        label="其余手指屈伸",
        max_motion_deg=max_stable_motion_deg,
        max_motion_ratio=max_stable_motion_ratio,
        min_travel_deg=min_flex_travel_deg,
    )
    validate_flex_capture(
        thumb_flex,
        THUMB_FLEX_INDICES,
        disabled_channels=disabled,
        label="大拇指屈伸",
        max_motion_deg=max_stable_motion_deg,
        max_motion_ratio=max_stable_motion_ratio,
        min_travel_deg=min_flex_travel_deg,
    )
    validate_lateral_capture(
        finger_lateral,
        FINGER_LATERAL_INDICES,
        disabled_channels=disabled,
        min_travel_deg=lateral_minimum,
    )
    validate_lateral_capture(
        thumb_lateral,
        THUMB_LATERAL_INDICES,
        disabled_channels=disabled,
        min_travel_deg=lateral_minimum,
    )

    flex_endpoints: dict[str, float] = {}
    flex_targets: dict[str, float] = {}
    lateral_ranges: dict[str, tuple[float, float]] = {}
    lateral_targets: dict[str, tuple[float, float]] = {}

    finger_flex_layers = (
        (6, 7, 8, 9),
        (11, 12, 13, 14),
        (16, 17, 18, 19),
    )
    for layer, glove_indices in enumerate(finger_flex_layers):
        for glove_index in glove_indices:
            joint = f"J{glove_index + 1}"
            if joint in disabled:
                continue
            flex_endpoints[joint] = finger_flex.values_deg[glove_index]
            target = math.radians(finger_targets[layer])
            flex_targets[joint] = -target if joint in inverted else target

    for layer, glove_index in enumerate(THUMB_FLEX_INDICES):
        joint = f"J{glove_index + 1}"
        if joint in disabled:
            continue
        flex_endpoints[joint] = thumb_flex.values_deg[glove_index]
        target = math.radians(thumb_targets[layer])
        flex_targets[joint] = -target if joint in inverted else target

    for glove_index in FINGER_LATERAL_INDICES:
        joint = f"J{glove_index + 1}"
        if joint in disabled:
            continue
        low = finger_lateral.minimum_deg[glove_index]
        high = finger_lateral.maximum_deg[glove_index]
        lateral_ranges[joint] = (
            low if low <= -lateral_minimum else 0.0,
            high if high >= lateral_minimum else 0.0,
        )
        output = math.radians(lateral_target)
        lateral_targets[joint] = (
            (output, -output) if joint in inverted else (-output, output)
        )

    joint = "J1"
    if joint not in disabled:
        low = thumb_lateral.minimum_deg[0]
        high = thumb_lateral.maximum_deg[0]
        lateral_ranges[joint] = (
            low if low <= -lateral_minimum else 0.0,
            high if high >= lateral_minimum else 0.0,
        )
        negative_output = math.radians(
            -min(lateral_target, THUMB_LATERAL_NEGATIVE_TARGET_DEG)
        )
        positive_output = math.radians(lateral_target)
        lateral_targets[joint] = (
            (positive_output, negative_output)
            if joint in inverted
            else (negative_output, positive_output)
        )

    return build_profile_from_captures(
        hand=hand,
        cs_by_joint=cs_by_joint,
        flex_endpoints_deg=flex_endpoints,
        flex_targets_rad=flex_targets,
        lateral_ranges_deg=lateral_ranges,
        lateral_targets_rad=lateral_targets,
        disabled_channels=disabled,
        deadband_deg=deadband_deg,
        zero_offsets_deg=zero_offsets,
    )


def _parse_joint_set(text: str) -> set[str]:
    channels = {
        part.strip().upper()
        for part in text.split(",")
        if part.strip()
    }
    valid = {f"J{index}" for index in range(1, 21)}
    unknown = channels - valid
    if unknown:
        raise ValueError(f"未知通道: {', '.join(sorted(unknown))}")
    return channels


def _enabled_digit_names(
    indices: Sequence[int],
    disabled_channels: Iterable[str],
) -> tuple[str, ...]:
    """按手指顺序列出一组采样通道实际需要动作的手指。"""

    disabled = {str(channel).upper() for channel in disabled_channels}
    active_digits = {
        index % len(DIGIT_NAMES)
        for index in indices
        if f"J{index + 1}" not in disabled
    }
    return tuple(
        name
        for digit, name in enumerate(DIGIT_NAMES)
        if digit in active_digits
    )


def _parse_triplet(text: str, label: str) -> tuple[float, float, float]:
    values = tuple(float(part.strip()) for part in text.split(","))
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} 必须是三个逗号分隔的有限数值")
    return values


def _read_window(
    host: str,
    port: int,
    duration_s: float,
    *,
    expected_mapping: Sequence[int] | None = None,
    profile: CalibrationProfile | None = None,
) -> CaptureWindow:
    if duration_s <= 0.0:
        raise ValueError("采样时间必须大于 0")

    window = read_encoder_window(host, port, duration_s, timeout=5.0)
    if profile is not None:
        profile.validate_stream(
            channels=window.channels,
            cs_by_joint=window.mapping,
        )
    else:
        if window.channels != CHANNEL_COUNT:
            raise ValueError(
                f"手套返回 {window.channels} 路，要求 {CHANNEL_COUNT} 路"
            )
        if (
            expected_mapping is not None
            and window.mapping != tuple(expected_mapping)
        ):
            raise ValueError("标定过程中板端 enc_map 发生变化")
    return window


def _print_zeroed_status(zeroed: bool) -> None:
    print(
        f"板端状态: zeroed={str(zeroed).lower()}"
        "（仅记录，不阻断）"
    )


def _print_stable(label: str, capture: StableCapture, indices: Sequence[int]) -> None:
    print(f"\n{label}：")
    for index in indices:
        print(
            f"  J{index + 1}: {capture.values_deg[index]:+8.3f}° "
            f"(motion80={capture.motion_deg[index]:.3f}°)"
        )


def _print_sweep(label: str, capture: SweepCapture, indices: Sequence[int]) -> None:
    print(f"\n{label}：")
    for index in indices:
        low = capture.minimum_deg[index]
        high = capture.maximum_deg[index]
        print(f"  J{index + 1}: [{low:+8.3f}°, {high:+8.3f}°]")


def _capture_zero_until_valid(
    *,
    prompt: str,
    host: str,
    port: int,
    duration_s: float,
    disabled_channels: Iterable[str],
    max_motion_deg: float,
) -> tuple[StableCapture, tuple[int, ...]]:
    """采集自然放松姿态，并拒绝采样窗口内明显移动的通道。"""

    disabled = {
        *(str(channel).upper() for channel in disabled_channels),
        "J21",
    }
    while True:
        input(prompt)
        window = _read_window(host, port, duration_s)
        _print_zeroed_status(window.zeroed)
        capture = summarize_stable_frames(window.frames)
        _print_stable(
            "自然放松零位",
            capture,
            tuple(range(CHANNEL_COUNT)),
        )
        moving = [
            f"J{index + 1}"
            for index, motion in enumerate(capture.motion_deg)
            if f"J{index + 1}" not in disabled
            and motion > max_motion_deg
        ]
        if moving:
            print(
                "\n零位采样不稳定："
                + ", ".join(moving)
                + "；保持自然放松和静止后只重采本步骤。\n",
                file=sys.stderr,
            )
            continue
        print("自然放松零位采样通过。\n")
        return capture, window.mapping


def _capture_stable_until_valid(
    *,
    prompt: str,
    label: str,
    indices: Sequence[int],
    host: str,
    port: int,
    duration_s: float,
    expected_mapping: Sequence[int] | None,
    disabled_channels: Iterable[str],
    max_motion_deg: float,
    max_motion_ratio: float,
    min_travel_deg: float,
    zero_offsets_deg: Sequence[float] = (0.0,) * CHANNEL_COUNT,
) -> tuple[StableCapture, tuple[int, ...]]:
    mapping = tuple(expected_mapping) if expected_mapping is not None else None
    while True:
        input(prompt)
        window = _read_window(
            host,
            port,
            duration_s,
            expected_mapping=mapping,
        )
        current_mapping = window.mapping
        if mapping is None:
            mapping = current_mapping
        _print_zeroed_status(window.zeroed)
        capture = summarize_stable_frames(window.frames)
        relative_capture = capture.relative_to(zero_offsets_deg)
        _print_stable(f"{label}（相对零位）", relative_capture, indices)
        try:
            validate_flex_capture(
                relative_capture,
                indices,
                disabled_channels=disabled_channels,
                label=label,
                max_motion_deg=max_motion_deg,
                max_motion_ratio=max_motion_ratio,
                min_travel_deg=min_travel_deg,
            )
        except ValueError as exc:
            print(
                f"\n当前步骤未通过：{exc}\n"
                "只需重新采当前步骤；保持姿态稳定后按 Enter，"
                "或按 Ctrl+C 取消。\n",
                file=sys.stderr,
            )
            continue
        print("本步骤通过。\n")
        return capture, mapping


def _capture_lateral_until_valid(
    *,
    prompt: str,
    label: str,
    indices: Sequence[int],
    host: str,
    port: int,
    duration_s: float,
    expected_mapping: Sequence[int],
    disabled_channels: Iterable[str],
    min_travel_deg: float,
    zero_offsets_deg: Sequence[float] = (0.0,) * CHANNEL_COUNT,
) -> SweepCapture:
    while True:
        input(prompt)
        print(f"开始扫角 {duration_s:.1f} 秒...")
        window = _read_window(
            host,
            port,
            duration_s,
            expected_mapping=expected_mapping,
        )
        _print_zeroed_status(window.zeroed)
        capture = summarize_sweep_frames(window.frames)
        relative_capture = capture.relative_to(zero_offsets_deg)
        _print_sweep(f"{label}（相对零位）", relative_capture, indices)
        try:
            validate_lateral_capture(
                relative_capture,
                indices,
                disabled_channels=disabled_channels,
                min_travel_deg=min_travel_deg,
            )
        except ValueError as exc:
            print(
                f"\n当前步骤未通过：{exc}\n"
                "只需重新采当前步骤；按 Enter 后重新扫角，"
                "或按 Ctrl+C 取消。\n",
                file=sys.stderr,
            )
            continue
        print("本步骤通过（单侧或双侧有效量程均可）。\n")
        return capture


def command_calibrate(args: argparse.Namespace) -> int:
    disabled = _parse_joint_set(args.disabled)
    invert_text = (
        args.invert
        if args.invert is not None
        else ",".join(DEFAULT_INVERTED_CHANNELS[args.hand])
    )
    inverted = _parse_joint_set(invert_text)
    finger_targets = _parse_triplet(
        args.finger_flex_target_deg,
        "--finger-flex-target-deg",
    )
    thumb_targets = (
        _parse_triplet(args.thumb_flex_target_deg, "--thumb-flex-target-deg")
        if args.thumb_flex_target_deg
        else None
    )
    positive_options = (
        ("--sample-seconds", args.sample_seconds),
        ("--sweep-seconds", args.sweep_seconds),
        ("--lateral-target-deg", args.lateral_target_deg),
        ("--min-flex-travel-deg", args.min_flex_travel_deg),
        ("--min-lateral-travel-deg", args.min_lateral_travel_deg),
    )
    for option, value in positive_options:
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{option} 必须是正有限数值")
    nonnegative_options = (
        ("--deadband-deg", args.deadband_deg),
        ("--max-stable-motion-deg", args.max_stable_motion_deg),
        ("--max-stable-motion-ratio", args.max_stable_motion_ratio),
    )
    for option, value in nonnegative_options:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{option} 必须是非负有限数值")

    output = (
        Path(args.output)
        if args.output
        else get_hand_resources(args.hand).calibration
    )

    print("数据手套 → Wuji 尺度标定")
    print(f"手型: {args.hand}  手套: {args.host}:{args.port}")
    print(f"屏蔽通道: {', '.join(sorted(disabled)) or '无'}")
    print(f"方向反转: {', '.join(sorted(inverted)) or '无'}")
    print(
        "板端 zeroed 状态仅记录、不作为门禁；"
        "本工具直接使用当前角度标定方向与尺度。"
    )
    print("先采自然放松零位；大拇指与其余可用手指完全分开采样。\n")
    finger_flex_names = "、".join(
        _enabled_digit_names(FINGER_FLEX_INDICES, disabled)
    )
    thumb_flex_names = "、".join(
        _enabled_digit_names(THUMB_FLEX_INDICES, disabled)
    )
    finger_lateral_names = "、".join(
        _enabled_digit_names(FINGER_LATERAL_INDICES, disabled)
    )
    thumb_lateral_names = "、".join(
        _enabled_digit_names(THUMB_LATERAL_INDICES, disabled)
    )

    zero_capture, mapping = _capture_zero_until_valid(
        prompt=(
            "步骤 1/5：自然放松零位\n"
            "手掌与全部手指保持自然放松并静止，按 Enter 采样..."
        ),
        host=args.host,
        port=args.port,
        duration_s=args.sample_seconds,
        disabled_channels=disabled,
        max_motion_deg=args.max_stable_motion_deg,
    )

    finger_flex, _ = _capture_stable_until_valid(
        prompt=(
            "步骤 2/5：其余可用手指屈伸\n"
            + (
                f"保持大拇指自然零位，仅将{finger_flex_names}的可用屈伸"
                "关节弯到希望映射为最大值的姿态并保持，按 Enter 采样..."
                if finger_flex_names
                else "本步骤屈伸通道均已屏蔽；保持全部手指自然位并静止，"
                "按 Enter 采样..."
            )
        ),
        label="其余手指屈伸",
        indices=FINGER_FLEX_INDICES,
        host=args.host,
        port=args.port,
        duration_s=args.sample_seconds,
        expected_mapping=mapping,
        disabled_channels=disabled,
        max_motion_deg=args.max_stable_motion_deg,
        max_motion_ratio=args.max_stable_motion_ratio,
        min_travel_deg=args.min_flex_travel_deg,
        zero_offsets_deg=zero_capture.values_deg,
    )

    thumb_flex, _ = _capture_stable_until_valid(
        prompt=(
            "\n步骤 3/5：大拇指屈伸（单独标定）\n"
            + (
                f"其余手指回到自然位，只将{thumb_flex_names}的可用屈伸"
                "关节弯到目标最大姿态并保持，按 Enter 采样..."
                if thumb_flex_names
                else "本步骤大拇指屈伸通道均已屏蔽；保持全部手指自然位"
                "并静止，按 Enter 采样..."
            )
        ),
        label="大拇指屈伸",
        indices=THUMB_FLEX_INDICES,
        host=args.host,
        port=args.port,
        duration_s=args.sample_seconds,
        expected_mapping=mapping,
        disabled_channels=disabled,
        max_motion_deg=args.max_stable_motion_deg,
        max_motion_ratio=args.max_stable_motion_ratio,
        min_travel_deg=args.min_flex_travel_deg,
        zero_offsets_deg=zero_capture.values_deg,
    )

    finger_lateral = _capture_lateral_until_valid(
        prompt=(
            "\n步骤 4/5：其余可用手指侧摆扫角\n"
            + (
                f"保持大拇指自然位和其余手指尽量伸直；按 Enter 后，"
                f"在计时窗口内依次只将{finger_lateral_names}扫到自然活动"
                "极限，单侧或双侧均可..."
                if finger_lateral_names
                else "本步骤侧摆通道均已屏蔽；保持全部手指自然位并静止，"
                "按 Enter 采样..."
            )
        ),
        label="其余手指侧摆扫角",
        indices=FINGER_LATERAL_INDICES,
        host=args.host,
        port=args.port,
        duration_s=args.sweep_seconds,
        expected_mapping=mapping,
        disabled_channels=disabled,
        min_travel_deg=args.min_lateral_travel_deg,
        zero_offsets_deg=zero_capture.values_deg,
    )

    thumb_lateral = _capture_lateral_until_valid(
        prompt=(
            "\n步骤 5/5：大拇指侧摆（单独标定）\n"
            + (
                f"其余手指保持自然位；按 Enter 后，只将"
                f"{thumb_lateral_names}扫到自然活动极限，有双侧活动就扫过"
                "双侧..."
                if thumb_lateral_names
                else "本步骤大拇指侧摆通道已屏蔽；保持全部手指自然位"
                "并静止，按 Enter 采样..."
            )
        ),
        label="大拇指侧摆扫角",
        indices=THUMB_LATERAL_INDICES,
        host=args.host,
        port=args.port,
        duration_s=args.sweep_seconds,
        expected_mapping=mapping,
        disabled_channels=disabled,
        min_travel_deg=args.min_lateral_travel_deg,
        zero_offsets_deg=zero_capture.values_deg,
    )

    profile = build_workflow_profile(
        hand=args.hand,
        cs_by_joint=mapping,
        finger_flex=finger_flex,
        thumb_flex=thumb_flex,
        finger_lateral=finger_lateral,
        thumb_lateral=thumb_lateral,
        disabled_channels=disabled,
        deadband_deg=args.deadband_deg,
        finger_flex_target_deg=finger_targets,
        thumb_flex_target_deg=thumb_targets,
        lateral_target_deg=args.lateral_target_deg,
        inverted_channels=inverted,
        max_stable_motion_deg=args.max_stable_motion_deg,
        max_stable_motion_ratio=args.max_stable_motion_ratio,
        min_flex_travel_deg=args.min_flex_travel_deg,
        min_lateral_travel_deg=args.min_lateral_travel_deg,
        zero_offsets_deg=zero_capture.values_deg,
    )
    profile.save(output)
    print(f"\n标定完成：{output}")
    print(
        "下一步先运行 dry-run 和 MuJoCo；"
        "真机必须按 docs/operations.md 的双终端流程启动。"
    )
    return 0


def command_inspect(args: argparse.Namespace) -> int:
    window = _read_window(
        args.host,
        args.port,
        args.seconds,
    )
    frames = window.frames
    stable = summarize_stable_frames(frames)
    sweep = summarize_sweep_frames(frames)
    duration_s = (frames[-1].timestamp_ns - frames[0].timestamp_ns) / 1e9
    rate_hz = (len(frames) - 1) / duration_s if duration_s > 0.0 else 0.0
    print(
        f"channels={CHANNEL_COUNT} "
        f"zeroed={str(window.zeroed).lower()} rate={rate_hz:.2f} Hz"
    )
    print(
        "cs_by_joint="
        + ",".join(str(value) for value in window.mapping)
    )
    print(
        f"sequence={frames[0].sequence}..{frames[-1].sequence} "
        f"dropped={frames[0].dropped}..{frames[-1].dropped}"
    )
    for index in range(CHANNEL_COUNT):
        print(
            f"J{index + 1:02d} median={stable.values_deg[index]:+8.3f}° "
            f"range=[{sweep.minimum_deg[index]:+8.3f},"
            f"{sweep.maximum_deg[index]:+8.3f}]°"
        )
    return 0
