#!/usr/bin/env python3
"""Load the Regrind RSL-RL actor and benchmark CPU or CUDA inference."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from ...data.reference import load_reference


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--h5", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    args = parser.parse_args(argv)
    if args.iterations < 1:
        parser.error("--iterations must be positive")
    if not args.model.is_file():
        parser.error(f"model not found: {args.model}")
    if not args.h5.is_file():
        parser.error(f"trajectory not found: {args.h5}")

    # Policy dependencies are optional; argument help must not require PyTorch.
    import torch
    from .actor import DIMS, frame_zero_observation, infer, load_actor

    torch.set_num_threads(1)
    started = time.perf_counter_ns()
    actor, mean, variance, iteration = load_actor(args.model, device=args.device)
    load_ms = (time.perf_counter_ns() - started) / 1e6
    observation = frame_zero_observation(load_reference(args.h5))

    for _ in range(20):
        infer(actor, mean, variance, observation)
    timings_ms = []
    for _ in range(args.iterations):
        started = time.perf_counter_ns()
        action = infer(actor, mean, variance, observation)
        timings_ms.append((time.perf_counter_ns() - started) / 1e6)

    print(
        json.dumps(
            {
                "device": str(mean.device),
                "device_name": torch.cuda.get_device_name(mean.device) if mean.is_cuda else "cpu",
                "input": f"{args.h5.resolve()}:frame0",
                "checkpoint": str(args.model.resolve()),
                "checkpoint_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
                "checkpoint_iteration": iteration,
                "obs_dim": DIMS[0],
                "action_dim": DIMS[-1],
                "load_ms": round(load_ms, 3),
                "inference_mean_ms": round(float(np.mean(timings_ms)), 3),
                "inference_p99_ms": round(float(np.percentile(timings_ms, 99)), 3),
                "within_20ms": bool(np.percentile(timings_ms, 99) < 20.0),
                "raw_action": action.tolist(),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
