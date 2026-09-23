"""Resource-location contracts shared by the PICO bare-hand -> SPD route.

This package deliberately has **no** third-party imports beyond the standard
library, and never imports rclpy or a business package:

* it is imported by ``tianji_description`` (model/home resources),
  ``pico2_hands`` and ``simulation``, so importing any of them from here would
  create a dependency cycle;
* the offline checks import :mod:`tianji_runtime` and must not need a ROS
  runtime.

Only resource location remains here. The hardware device/feedback, operator
keyboard, camera-stream and dataset-layout contracts lived for the acquisition
routes that this branch no longer contains.

ROS message types live in ``tianji_spd_interfaces``; conversion is owned by
the SPD publisher, not by this resource package.
"""

from .resources import (
    ResourceNotFound,
    controller_profile,
    controller_resource,
    install_prefix,
    native_executable,
    package_share,
    workspace,
)

__all__ = [
    "ResourceNotFound",
    "controller_profile",
    "controller_resource",
    "install_prefix",
    "native_executable",
    "package_share",
    "workspace",
]
