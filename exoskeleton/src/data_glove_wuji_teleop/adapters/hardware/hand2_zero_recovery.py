"""不依赖手套的 Hand2 右手安全零目标与 lease 恢复源。"""

from __future__ import annotations

import argparse
import math
import os
import time
from collections.abc import Sequence
from contextlib import ExitStack
from pathlib import Path

from ...domain.hand_target import DOF_ORDER, HandTarget
from ..runtime.simulation_session import SimulationLease
from ..simulation.mujoco_wuji_official import open_official_target_publisher
from ..transport.zmq_dof import DEFAULT_ZMQ_PORT


def validate_hand2_zero_positions(
    positions: Sequence[float],
    *,
    tolerance_rad: float = 0.10,
) -> float:
    """返回最大绝对角；拒绝不完整、非有限或尚未到零的 Hand2 状态。"""
    values = tuple(float(value) for value in positions)
    tolerance = float(tolerance_rad)
    if len(values) != 20 or not all(math.isfinite(value) for value in values):
        raise ValueError("Hand2 零位确认必须包含 20 个有限关节角")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("Hand2 零位确认容差必须为正有限数")
    maximum = max(abs(value) for value in values)
    if maximum > tolerance:
        raise ValueError(f"Hand2 关闭回零未确认：max_abs={maximum:.5f}rad")
    return maximum


def make_hand2_zero_target(*, source: str, sequence: int) -> HandTarget:
    if source not in ("official_wuji_retarget", "official_wuji_retarget_shutdown"):
        raise ValueError(f"不允许的 Hand2 零目标来源：{source}")
    return HandTarget(
        hand="right",
        values=(0.0,) * len(DOF_ORDER),
        source=source,
        sequence=int(sequence),
        timestamp_ns=time.time_ns(),
        dropped=0,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Hand2 关闭恢复专用安全零目标源"
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_ZMQ_PORT["right"])
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--ready-file", type=Path, required=True)
    return parser


def run(args: argparse.Namespace) -> int:
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        raise ValueError("安全零目标频率必须为正数")
    period = 1.0 / args.rate
    with ExitStack() as stack:
        stack.enter_context(SimulationLease(generation="v2", hand="right"))
        publisher = stack.enter_context(
            open_official_target_publisher(args.bind, args.port, hand="right")
        )
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text(
            f"pid={os.getpid()}\n",
            encoding="utf-8",
        )
        stack.callback(args.ready_file.unlink, missing_ok=True)
        print(
            f"Hand2 安全零恢复源已启动：tcp://{args.bind}:{args.port}",
            flush=True,
        )
        sequence = 0
        while True:
            publisher.publish(
                make_hand2_zero_target(
                    source="official_wuji_retarget",
                    sequence=sequence,
                )
            )
            sequence += 1
            time.sleep(period)


def main() -> None:
    try:
        raise SystemExit(run(build_parser().parse_args()))
    except KeyboardInterrupt:
        print("\nHand2 安全零恢复源停止。", flush=True)


if __name__ == "__main__":
    main()
