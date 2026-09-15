#!/usr/bin/env python3
"""Estimate empirical Cartesian H1 frequency responses from FRF CSV cases."""

import argparse
import csv
import json
from collections import defaultdict, namedtuple
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


FrfEstimate = namedtuple(
    "FrfEstimate", "frequency response magnitude_db phase_deg group_delay coherence input_power")


def estimate_frf(u, y, fs, nperseg=1024):
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    nperseg = min(int(nperseg), u.size)
    noverlap = nperseg // 2
    frequency, suu = signal.welch(u, fs=fs, window="hann", nperseg=nperseg,
                                  noverlap=noverlap, detrend="linear")
    _, syu = signal.csd(u, y, fs=fs, window="hann", nperseg=nperseg,
                        noverlap=noverlap, detrend="linear")
    _, syy = signal.welch(y, fs=fs, window="hann", nperseg=nperseg,
                          noverlap=noverlap, detrend="linear")
    response = syu / np.maximum(suu, np.finfo(float).tiny)
    coherence = np.abs(syu) ** 2 / np.maximum(suu * syy, np.finfo(float).tiny)
    phase = np.unwrap(np.angle(response))
    omega = 2.0 * np.pi * frequency
    group_delay = -np.gradient(phase, omega, edge_order=1)
    return FrfEstimate(frequency, response,
                       20.0 * np.log10(np.maximum(np.abs(response), 1e-15)),
                       np.rad2deg(phase), group_delay,
                       np.clip(coherence, 0.0, 1.0), suu)


def valid_bins(estimate, coherence_threshold=0.8):
    # A high coherence number is not sufficient when the chirp has virtually
    # no energy in a Welch bin. Exclude bins more than 40 dB below peak input.
    power_floor = np.max(estimate.input_power) * 1.0e-4
    return ((estimate.coherence >= coherence_threshold) &
            (estimate.input_power >= power_floor))


def minus_3db_bandwidth(estimate, coherence_threshold=0.8):
    valid = (estimate.frequency > 0.0) & valid_bins(estimate, coherence_threshold)
    if not np.any(valid):
        return float("nan")
    low = valid & (estimate.frequency <= 0.5)
    baseline = np.median(estimate.magnitude_db[low]) if np.any(low) else estimate.magnitude_db[valid][0]
    indices = np.flatnonzero(valid & (estimate.magnitude_db <= baseline - 3.0))
    return float(estimate.frequency[indices[0]]) if indices.size else float("nan")


def _metadata(path):
    values = {}
    with path.open() as stream:
        for line in stream:
            if not line.startswith("#"):
                break
            key, value = line[1:].strip().split("=", 1)
            values[key] = value
    return values


def analyze_case(path, nperseg=1024):
    metadata = _metadata(path)
    with path.open() as stream:
        csv_lines = [line for line in stream if not line.startswith("#")]
    data = np.genfromtxt(csv_lines, delimiter=",", names=True)
    dt = float(metadata.get("dt", "0.005"))
    warmup = float(metadata.get("warmup", "2.0"))
    chirp = float(metadata.get("chirp", "20.0"))
    time = data["time_s"]
    mask = (time >= warmup) & (time < warmup + chirp)
    # All pipelines receive the same desired Cartesian perturbation.  The
    # target_input column is diagnostic (post-retarget/post-OTG) and can be
    # intentionally zero in nonlinear hold mode, so it must not redefine the
    # experiment input.
    estimate = estimate_frf(data["input"][mask], data["output"][mask],
                            1.0 / dt, min(nperseg, int(np.sum(mask))))
    band = (estimate.frequency >= 0.2) & (estimate.frequency <= 15.0)
    valid = band & valid_bins(estimate)
    low = valid & (estimate.frequency <= 0.5)
    delay = valid & (estimate.frequency >= 1.0) & (estimate.frequency <= 5.0)
    low_gain = float(np.median(10.0 ** (estimate.magnitude_db[low] / 20.0))) if np.any(low) else float("nan")
    normalized = estimate.magnitude_db - (np.median(estimate.magnitude_db[low]) if np.any(low) else 0.0)
    metrics = dict(metadata)
    metrics.update({
        "file": str(path), "low_frequency_gain": low_gain,
        "resonant_peak_db": float(np.max(normalized[valid])) if np.any(valid) else float("nan"),
        "bandwidth_hz": minus_3db_bandwidth(estimate),
        "group_delay_median_ms": float(1000.0 * np.median(estimate.group_delay[delay])) if np.any(delay) else float("nan"),
        "group_delay_p95_ms": float(1000.0 * np.percentile(estimate.group_delay[delay], 95)) if np.any(delay) else float("nan"),
        "coherence_valid_fraction": float(np.mean(valid[band])),
        "effective_frequency_max_hz": float(np.max(estimate.frequency[valid])) if np.any(valid) else float("nan"),
        "control_failures": int(np.sum(data["accepted"] < 0.5)),
        "position_violations": int(np.sum(data["position_violation"] > 0.5)),
        "velocity_violations": int(np.sum(data["velocity_violation"] > 0.5)),
        "acceleration_violations": int(np.sum(data["acceleration_violation"] > 0.5)),
        "jerk_violations": int(np.sum(data["jerk_violation"] > 0.5)),
        "solve_time_p99_us": float(np.percentile(data["solve_time_us"], 99)),
    })
    return metrics, estimate


def analyze_directory(input_root, output_root=None):
    input_root = Path(input_root)
    output_root = Path(output_root or input_root)
    figures = output_root / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    results = []
    grouped = defaultdict(list)
    for path in sorted((input_root / "raw").glob("*.csv")):
        metrics, estimate = analyze_case(path)
        results.append(metrics)
        grouped[(metrics.get("arm"), metrics.get("channel"))].append((metrics, estimate))
    if not results:
        raise RuntimeError(f"no raw CSV files under {input_root / 'raw'}")
    keys = sorted({key for row in results for key in row})
    with (output_root / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader(); writer.writerows(results)
    for (arm, channel), cases in grouped.items():
        fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        by_algorithm = defaultdict(list)
        for metrics, estimate in cases:
            by_algorithm[metrics["algorithm"]].append(estimate)
        for algorithm, estimates in sorted(by_algorithm.items()):
            frequency = estimates[0].frequency
            arrays = []
            for estimate in estimates:
                arrays.append(np.interp(frequency, estimate.frequency, estimate.magnitude_db))
            axes[0].semilogx(frequency, np.median(arrays, axis=0), label=algorithm)
            axes[1].semilogx(frequency, np.median([np.interp(frequency, e.frequency, e.phase_deg) for e in estimates], axis=0))
            axes[2].semilogx(frequency, 1000.0 * np.median([np.interp(frequency, e.frequency, e.group_delay) for e in estimates], axis=0))
            axes[3].semilogx(frequency, np.median([np.interp(frequency, e.frequency, e.coherence) for e in estimates], axis=0))
        axes[0].set_ylabel("Magnitude [dB]"); axes[0].legend(fontsize=7)
        axes[1].set_ylabel("Phase [deg]")
        axes[2].set_ylabel("Group delay [ms]")
        axes[3].set_ylabel("Coherence"); axes[3].set_xlabel("Frequency [Hz]")
        axes[3].axhline(0.8, color="gray", linestyle="--")
        for axis in axes: axis.grid(True, which="both", alpha=0.3); axis.set_xlim(0.2, 15.0)
        fig.suptitle(f"Cartesian empirical FRF: {arm} {channel}")
        fig.tight_layout(); fig.savefig(figures / f"{arm}__{channel}__bode.png", dpi=160); plt.close(fig)
    (output_root / "metrics.json").write_text(json.dumps(results, indent=2, allow_nan=True))
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", required=True)
    parser.add_argument("--output-root")
    args = parser.parse_args()
    analyze_directory(args.input_root, args.output_root)


if __name__ == "__main__":
    main()
