"""Lightweight contracts shared by the Tianji hardware adapters.

This package deliberately has **no** third-party imports beyond the standard
library, and never imports rclpy or a business package:

* it is imported by ``tianji_controller`` (executor) and by
  ``wuji_controller`` (hand adapter), so importing either one from here would
  create a dependency cycle;
* the offline tools (compression, dataset validation, config checks) import
  :mod:`tianji_runtime` and must not need a ROS runtime.

ROS message types live in ``tianji_interfaces``; conversion between these
contracts and those messages happens in the nodes, not here.
"""

from .device import Feedback, LockedDevice, advances, positions, fresh
from .hand2 import SERVICE_MODULE, Hand2Retargeter, Hand2ServiceError
from .keyboard import OperatorKeyboard
from .resources import (
    ResourceNotFound,
    compressed_dir,
    config_path,
    control_prefix,
    controller_profile,
    dataset_dir,
    install_prefix,
    native_executable,
    package_share,
    profiles_dir,
    vendor_path,
    workspace,
)
from .constants import (
    ARMS_COUNT,
    CAMERA_FPS,
    CAMERA_ROLES,
    DEVICES,
    HAND_COUNT,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    RGB_SHAPE,
    STATE_DIM,
    STATE_RATE_HZ,
)

__all__ = [
    "Feedback",
    "Hand2Retargeter",
    "Hand2ServiceError",
    "LockedDevice",
    "OperatorKeyboard",
    "ResourceNotFound",
    "SERVICE_MODULE",
    "compressed_dir",
    "config_path",
    "control_prefix",
    "controller_profile",
    "dataset_dir",
    "install_prefix",
    "native_executable",
    "package_share",
    "profiles_dir",
    "vendor_path",
    "workspace",
    "positions",
    "advances",
    "fresh",
    "ARMS_COUNT",
    "CAMERA_FPS",
    "CAMERA_ROLES",
    "DEVICES",
    "HAND_COUNT",
    "IMAGE_HEIGHT",
    "IMAGE_WIDTH",
    "RGB_SHAPE",
    "STATE_DIM",
    "STATE_RATE_HZ",
]
