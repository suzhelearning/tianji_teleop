#!/usr/bin/env python3
"""Compatibility API for the historical left-arm geometry module."""

from __future__ import annotations

from pico_arm_geometry_core import (
    ArmGeometryGate as LeftArmGeometryGate,
    ArmGeometryResult as LeftArmGeometryResult,
    ArmSide,
    ArmStaticPoseGate as LeftArmStaticPoseGate,
    ArmStaticPoseResult as LeftArmStaticPoseResult,
    ArmTwoPoseGate as LeftArmTwoPoseGate,
    ArmTwoPoseResult as LeftArmTwoPoseResult,
    CircleFit3d,
    fit_circle_3d,
    solve_arm_geometry,
    solve_arm_static_pose_geometry,
    solve_arm_two_pose_geometry,
)


def solve_left_arm_static_pose_geometry(*args, **kwargs) -> LeftArmStaticPoseResult:
    """Call the side-neutral static-pose solver with the historical left side."""

    return solve_arm_static_pose_geometry(*args, side=ArmSide.LEFT, **kwargs)


def solve_left_arm_two_pose_geometry(*args, **kwargs) -> LeftArmTwoPoseResult:
    """Call the side-neutral two-pose solver with the historical left side."""

    return solve_arm_two_pose_geometry(*args, side=ArmSide.LEFT, **kwargs)


def solve_left_arm_geometry(*args, **kwargs) -> LeftArmGeometryResult:
    """Call the side-neutral dynamic solver with the historical left side."""

    return solve_arm_geometry(*args, side=ArmSide.LEFT, **kwargs)


__all__ = [
    "CircleFit3d",
    "LeftArmGeometryGate",
    "LeftArmGeometryResult",
    "LeftArmStaticPoseGate",
    "LeftArmStaticPoseResult",
    "LeftArmTwoPoseGate",
    "LeftArmTwoPoseResult",
    "fit_circle_3d",
    "solve_left_arm_geometry",
    "solve_left_arm_static_pose_geometry",
    "solve_left_arm_two_pose_geometry",
]
