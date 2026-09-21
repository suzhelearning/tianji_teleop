#!/usr/bin/env python3
"""Run the deterministic Cartesian FRF factorial benchmark in parallel."""

import argparse
import concurrent.futures
import csv
import json
import os
import subprocess
import sys
from collections import namedtuple
from datetime import datetime
from pathlib import Path


ALGORITHMS = [
    "hierarchical_qp",
    "spark_upper_qpoases_velocity_qp",
    "spark_upper_qpoases_cartesian_otg_velocity_qp",
    "spark_upper_qpoases_feedforward_velocity_qp",
]
ARMS = ["left", "right"]
WORKING_POINTS = ["center", "x_negative", "x_positive", "y_negative",
                  "y_positive", "z_negative", "z_positive"]
CHANNELS = ["x", "y", "z", "rx", "ry", "rz"]


class Case(namedtuple("CaseBase", "algorithm arm working_point channel")):
    __slots__ = ()

    @property
    def slug(self):
        return "__".join(self)


def expand_cases(algorithms, arms, working_points, channels):
    return [Case(a, arm, point, channel) for a in algorithms for arm in arms
            for point in working_points for channel in channels]


def case_complete(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size < 40:
        return False
    try:
        with path.open() as stream:
            rows = csv.DictReader(line for line in stream if not line.startswith("#"))
            first = next(rows)
        return {"sample", "time_s", "input", "output", "accepted"}.issubset(first)
    except (OSError, StopIteration, csv.Error):
        return False


def _run_case(payload):
    case, paths, durations = payload
    output = paths["raw"] / f"{case.slug}.csv"
    temporary = output.with_suffix(".csv.tmp")
    command = [
        str(paths["binary"]), "--config", str(paths["config"]),
        "--model", str(paths["model"]), "--urdf", str(paths["urdf"]),
        "--algorithm", case.algorithm, "--arm", case.arm,
        "--working-point", case.working_point, "--channel", case.channel,
        "--warmup", str(durations[0]), "--chirp", str(durations[1]),
        "--settle", str(durations[2]), "--output", str(temporary),
    ]
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode == 0 and case_complete(temporary):
        temporary.replace(output)
        return case.slug, True, ""
    temporary.unlink(missing_ok=True)
    return case.slug, False, (result.stderr or result.stdout)[-4000:]


def _list(value, default):
    return value.split(",") if value else list(default)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("--algorithms")
    parser.add_argument("--arms")
    parser.add_argument("--working-points")
    parser.add_argument("--channels")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--output-root")
    parser.add_argument("--binary", default="build/tianji_cartesian_frf_benchmark")
    parser.add_argument("--config", default="config/qp_ik_pico_teleop.yaml")
    parser.add_argument("--model", default="models/marvin_m6_qp_test.xml")
    parser.add_argument("--urdf", default="models/marvin_m6_s_ccs_696_v4_local.urdf")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    if args.smoke and not any((args.algorithms, args.arms, args.working_points, args.channels)):
        algorithms = [ALGORITHMS[0], ALGORITHMS[-1]]
        arms, points, channels = ["left"], ["center"], ["x"]
    else:
        algorithms = _list(args.algorithms, ALGORITHMS)
        arms = _list(args.arms, ARMS)
        points = _list(args.working_points, WORKING_POINTS)
        channels = _list(args.channels, CHANNELS)
    cases = expand_cases(algorithms, arms, points, channels)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(args.output_root) if args.output_root else project / "benchmark_results" / f"cartesian_frf_{stamp}"
    raw = root / "raw"; raw.mkdir(parents=True, exist_ok=True)
    paths = {"raw": raw, "binary": (project / args.binary).resolve(),
             "config": (project / args.config).resolve(), "model": (project / args.model).resolve(),
             "urdf": (project / args.urdf).resolve()}
    durations = (0.1, 1.0, 0.1) if args.smoke else (2.0, 20.0, 2.0)
    pending = [case for case in cases if not (args.resume and case_complete(raw / f"{case.slug}.csv"))]
    failures = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(_run_case, (case, paths, durations)) for case in pending]
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            slug, ok, detail = future.result()
            print(f"[{index}/{len(futures)}] {'PASS' if ok else 'FAIL'} {slug}", flush=True)
            if not ok: failures.append({"case": slug, "detail": detail})
    manifest = {"cases": len(cases), "executed": len(pending), "failures": failures,
                "durations": durations, "jobs": args.jobs}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2))
    if failures:
        print(f"{len(failures)} cases failed; see {root / 'manifest.json'}", file=sys.stderr)
        return 2
    subprocess.run([sys.executable, str(project / "scripts/analyze_cartesian_frf.py"),
                    "--input-root", str(root)], check=True)
    (root / "README.md").write_text(
        "# Cartesian FRF benchmark\n\n"
        f"Cases: {len(cases)}\n\nJobs: {args.jobs}\n\n"
        "FRF uses H1 and excludes coherence below 0.8 from summary metrics.\n")
    print(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
