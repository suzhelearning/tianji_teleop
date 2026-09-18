"""Read live Motive and run shadow inference; never publish robot control."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from ...data.reference import RegrindReference, load_reference
from .tracking import RegrindMotiveSample, RegrindMotiveTracker, open_mocap_session


HAMMER_START_POSITION_TOLERANCE_M = 0.02
HAMMER_START_ORIENTATION_TOLERANCE_DEG = 10.0


def _reference_speed(value: str) -> float:
    speed = float(value)
    if not 0.0 < speed <= 1.0:
        raise argparse.ArgumentTypeError("--reference-speed must be finite and in (0, 1]")
    return speed


def hammer_alignment(
    reference: RegrindReference, sample: RegrindMotiveSample, start_frame: int = 0,
) -> dict[str, float | bool]:
    """Original gate: absolute hammer pose in calibrated Motive/training world.

    No wrist-relative registration is applied: registering away the initial
    object error would invalidate this physical placement check.
    """
    if sample.hammer_xyzw is None:
        raise ValueError("Regrind alignment requires a tracked hammer")
    position_error = float(np.linalg.norm(sample.hammer_xyzw[:3] - reference.object_pos[start_frame]))
    expected = Rotation.from_quat(np.roll(reference.object_quat_wxyz[start_frame], -1))
    actual = Rotation.from_quat(sample.hammer_xyzw[3:])
    orientation_error = float(np.rad2deg(np.linalg.norm((expected.inv() * actual).as_rotvec())))
    return {
        "hammer_start_position_error_mm": round(position_error * 1000.0, 3),
        "hammer_start_orientation_error_deg": round(orientation_error, 3),
        "real_start_preflight_passed": (
            position_error <= HAMMER_START_POSITION_TOLERANCE_M
            and orientation_error <= HAMMER_START_ORIENTATION_TOLERANCE_DEG
        ),
    }


def _report(value: dict) -> None:
    print(json.dumps(value, separators=(",", ":"), allow_nan=False), flush=True)


def _fresh_sample(live: RegrindMotiveTracker, stale_s: float) -> RegrindMotiveSample:
    sample = live.latest()
    if live.error:
        raise RuntimeError(live.error)
    if sample is None or sample.hammer_xyzw is None:
        raise RuntimeError("Motive wrist or hammer tracking invalid")
    age_s = time.monotonic() - sample.received_at
    if age_s > stale_s:
        raise RuntimeError(f"Motive frame stale: {age_s:.3f}s")
    return sample


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--reference-speed", type=_reference_speed, default=1.0)
    parser.add_argument("--endpoint", default=os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    parser.add_argument("--wrist-name", default="right_wrist")
    parser.add_argument("--hammer-name", default="hammer")
    parser.add_argument("--stale-s", type=float, default=0.25)
    parser.add_argument("--wait-s", type=float, default=10.0)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="read-only reference/live alignment; no policy inference")
    args = parser.parse_args(argv)
    if not args.reference.is_file():
        parser.error("--reference must be an existing file")
    if not (args.viewer or args.preflight_only) and (args.model is None or not args.model.is_file()):
        parser.error("shadow inference requires an existing --model")
    if args.viewer and args.preflight_only:
        parser.error("--viewer and --preflight-only are mutually exclusive")
    if args.rate != 50.0:
        parser.error("Regrind policy rate must remain training-exact at 50 Hz")
    if any(not np.isfinite(value) or value <= 0 for value in (args.stale_s, args.wait_s)):
        parser.error("--stale-s and --wait-s must be finite and positive")
    if args.print_every < 1:
        parser.error("--print-every must be positive")
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be finite and positive")

    session = None
    live = None
    try:
        reference = load_reference(args.reference)
        if not 0 <= args.start_frame < reference.frame_count - 1:
            parser.error(f"--start-frame must be in [0, {reference.frame_count - 2}]")
        policy = None
        if not (args.viewer or args.preflight_only):
            import torch
            from .runtime import RegrindPolicy
            torch.set_num_threads(1)
            policy = RegrindPolicy(
                args.model, reference, device=args.device, start_frame=args.start_frame,
                reference_speed=args.reference_speed,
            )
        session = open_mocap_session(args.endpoint)
        live = RegrindMotiveTracker(session, wrist_name=args.wrist_name, hammer_name=args.hammer_name)
        deadline = time.monotonic() + args.wait_s
        sample = live.latest()
        while sample is None and not live.error and time.monotonic() < deadline:
            time.sleep(0.01)
            sample = live.latest()
        if live.error:
            raise RuntimeError(live.error)
        if sample is None:
            raise RuntimeError("timed out waiting for valid Motive wrist+hammer poses")
        sample = _fresh_sample(live, args.stale_s)
        alignment = hammer_alignment(reference, sample, args.start_frame)
        _report({
            "event": "started", "mode": "live_motive_alignment" if args.viewer else "live_motive_shadow_inference",
            "publishes_control": False, "checkpoint_iteration": policy.iteration if policy else None,
            "device": policy.device if policy else None, "rate_hz": args.rate,
            "reference_speed": args.reference_speed,
            "frames": reference.frame_count - args.start_frame - 1,
            "start_frame": args.start_frame,
            "hand_joint_observation": (
                "previous_policy_target_assuming_perfect_tracking" if policy else "not_used"
            ),
            **alignment,
        })
        if args.preflight_only:
            return 0 if alignment["real_start_preflight_passed"] else 1
        if args.viewer:
            from ...replay.alignment import run_alignment
            return run_alignment(args, reference, live)

        # This deliberately explicit assumption is exclusive to SHADOW mode.
        # The reusable runtime always consumes the hand feedback supplied by its caller.
        assumed_joints = reference.joints[args.start_frame].copy()
        policy.reset(sample.wrist_xyzw, assumed_joints)
        started = next_tick = time.monotonic()
        ticks = 0
        while not policy.complete:
            if args.duration is not None and time.monotonic() - started >= args.duration:
                break
            next_tick += 1.0 / args.rate
            sample = _fresh_sample(live, args.stale_s)
            age_s = time.monotonic() - sample.received_at
            target = policy.step(sample.wrist_xyzw, sample.hammer_xyzw, assumed_joints)
            wrist_target = target.wrist_poses["right"]
            assumed_joints = target.hand_joints["right"]
            if ticks % args.print_every == 0:
                _report({
                    "frame": target.index, "motive_frame": sample.frame_number,
                    "motive_age_ms": round(age_s * 1000.0, 3),
                    "inference_ms": round(policy.inference_ms, 3),
                    "wrist_pos": sample.wrist_xyzw[:3].tolist(),
                    "wrist_quat_wxyz": np.roll(sample.wrist_xyzw[3:], 1).tolist(),
                    "hammer_pos": sample.hammer_xyzw[:3].tolist(),
                    "hammer_quat_wxyz": np.roll(sample.hammer_xyzw[3:], 1).tolist(),
                    "raw_action": policy.raw_action.tolist(),
                    "target_wrist_pos": wrist_target[:3].tolist(),
                    "target_wrist_quat_wxyz": np.roll(wrist_target[3:], 1).tolist(),
                    "target_joints": assumed_joints.tolist(),
                    "reference_progress": policy.reference_progress,
                })
            ticks += 1
            time.sleep(max(0.0, next_tick - time.monotonic()))
        _report({
            "event": "completed" if policy.complete else "duration_elapsed",
            "frames": ticks, "reference_progress": policy.reference_progress,
            "reference_complete": policy.complete, "publishes_control": False,
        })
        return 0
    except KeyboardInterrupt:
        _report({"event": "interrupted", "publishes_control": False})
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        _report({"event": "error", "message": str(exc), "publishes_control": False})
        return 1
    finally:
        try:
            if live is not None:
                live.close()
        finally:
            if session is not None:
                session.close()


if __name__ == "__main__":
    raise SystemExit(main())
