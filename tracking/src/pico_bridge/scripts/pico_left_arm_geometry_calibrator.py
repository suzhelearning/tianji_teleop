#!/usr/bin/env python3
"""Compatibility entry point for left PICO arm geometry calibration."""

from __future__ import annotations

from pico_arm_geometry_calibrator import (
    ArmCaptureBuffer as LeftArmCaptureBuffer,
    CaptureSample,
    PendingRawSample,
    PicoArmGeometryCalibrator as PicoLeftArmGeometryCalibrator,
    StageDefinition,
    _array,
    _countdown,
    _spin_for,
    atomic_write_bundle as _atomic_write_bundle,
    candidate_from_result,
    default_stage_sequence as _default_stage_sequence,
    main_for_side,
    stable_suffix,
    wrist_from_palm,
)
from pico_arm_geometry_core import ArmSide


def default_stage_sequence() -> tuple[StageDefinition, ...]:
    return _default_stage_sequence(ArmSide.LEFT)


def _candidate_from_result(result, **kwargs):
    return candidate_from_result(result, side=ArmSide.LEFT, **kwargs)


def atomic_write_bundle(output_dir, capture, candidate, gate_report) -> None:
    left_candidate = dict(candidate)
    left_candidate.setdefault("side", ArmSide.LEFT.value)
    _atomic_write_bundle(output_dir, capture, left_candidate, gate_report)


def main() -> None:
    main_for_side("left")


if __name__ == "__main__":
    main()
