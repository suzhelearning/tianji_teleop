"""Enumerate RealSense devices and validate the configured stream profiles.

This is a read-only check: it creates an ``rs.context()``, reads device identity
and supported stream modes, and closes. It never opens a pipeline, so it cannot
disturb a driver that is already streaming.

Run as a Pixi task:
    pixi run inspect-cameras
"""

from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import sys
import tempfile
from dataclasses import dataclass

from tianji_runtime import config_path
from tianji_runtime.constants import CAMERA_FPS, CAMERA_ROLES, IMAGE_HEIGHT, IMAGE_WIDTH

# D405 exposes colour through the depth sensor. Product-line "D400" does not
# identify this model: use the SDK device name, not product_line.
DEPTH_MODULE_COLOR_PRODUCTS = ("D405",)
TARGET_PROFILE = f"{IMAGE_WIDTH},{IMAGE_HEIGHT},{CAMERA_FPS}"
TARGET_FORMAT = "RGB8"


@dataclass(frozen=True)
class CameraProfile:
    """Verified identity and stream capability of one configured camera."""

    role: str
    serial: str
    product: str
    usb_type: str
    uses_depth_module_color: bool
    supported_profiles: tuple[str, ...]

    def describe(self) -> str:
        channel = "depth_module" if self.uses_depth_module_color else "rgb_camera"
        return (f"{self.role}: serial={self.serial} product={self.product} "
                f"usb={self.usb_type or 'n/a'} {channel}.color_profile={TARGET_PROFILE} "
                f"{channel}.color_format={TARGET_FORMAT}")


class PreflightError(RuntimeError):
    """A configured camera is missing or cannot produce the required stream."""


def load_camera_selection(path=None):
    """Return the enabled ``{role: serial}`` mapping from the collection config."""
    from data_collector.config import load_collection_config

    _, enabled = load_collection_config(path or config_path("collect_real.json"))
    return enabled


def uses_depth_module_color(device_name: str) -> bool:
    return any(model in device_name.upper() for model in DEPTH_MODULE_COLOR_PRODUCTS)


def _profile_for(role: str, serial: str, device) -> CameraProfile:
    rs = _rs()
    product = device.get_info(rs.camera_info.name).strip()
    if not product:
        raise PreflightError(f"{role}: serial {serial} has no device name")
    usb_type = (device.get_info(rs.camera_info.usb_type_descriptor).strip()
                if device.supports(rs.camera_info.usb_type_descriptor) else "")
    return CameraProfile(
        role, serial, product, usb_type, uses_depth_module_color(product), ())


def _rs():
    import pyrealsense2 as rs
    return rs


def validate_devices(selection, validate_modes: bool = True):
    """Resolve every configured serial to a device and validate its stream mode."""
    import pyrealsense2 as rs

    context = rs.context()
    devices = {}
    for device in context.query_devices():
        devices[device.get_info(rs.camera_info.serial_number)] = device

    profiles = {}
    problems = []
    for role in CAMERA_ROLES:
        serial = selection.get(role)
        if serial is None:
            continue
        device = devices.get(serial)
        if device is None:
            problems.append(f"{role}: serial {serial} is not attached")
            continue
        profile = _profile_for(role, serial, device)
        if validate_modes:
            sensor = _color_sensor(device, rs, profile.uses_depth_module_color)
            if sensor is None:
                problems.append(f"{role}: no colour sensor available on serial {serial}")
                continue
            if not _supports(sensor, rs, rs.format.rgb8, IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_FPS):
                problems.append(
                    f"{role}: serial {serial} does not expose "
                    f"{IMAGE_WIDTH}x{IMAGE_HEIGHT}@{CAMERA_FPS} RGB8. The official driver "
                    "would silently fall back to a different mode, which must not be recorded.")
                continue
        profiles[role] = profile
        print(f"   ok {profile.describe()}")

    if problems:
        raise PreflightError("; ".join(problems))
    return profiles


def _color_sensor(device, rs, depth_module: bool):
    """The sensor that carries the configured colour stream."""
    if depth_module:
        # D405's combined sensor is named "Stereo Module", not "Depth".
        return device.first_depth_sensor()
    for sensor in device.sensors:
        if any(profile.stream_type() == rs.stream.color
               for profile in sensor.get_stream_profiles()):
            return sensor
    return None


def _supports(sensor, rs, fmt, width: int, height: int, fps: int) -> bool:
    for profile in sensor.get_stream_profiles():
        if profile.stream_type() != rs.stream.color:
            continue
        video = profile.as_video_stream_profile()
        if (profile.format() == fmt and video.width() == width
                and video.height() == height and video.fps() == fps):
            return True
    return False


class CameraLocks:
    """All-or-nothing process-owned serial locks; never unlink a live lock file."""

    def __init__(self, serials):
        self._files = {}
        directory = Path(tempfile.gettempdir()) / f"tianji-cameras-{os.getuid()}"
        directory.mkdir(mode=0o700, exist_ok=True)
        if directory.is_symlink() or directory.stat().st_uid != os.getuid():
            raise RuntimeError(f"unsafe camera lock directory: {directory}")
        try:
            for serial in sorted(set(serials)):
                if not serial.isascii() or not serial.isalnum():
                    raise ValueError(f"invalid camera serial {serial!r}")
                fd = os.open(directory / f"{serial}.lock",
                             os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
                handle = os.fdopen(fd, "r+")
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as error:
                    handle.close()
                    raise RuntimeError(f"camera serial {serial} already has an owner") from error
                self._files[serial] = handle
        except BaseException:
            self.close()
            raise

    def release(self, serial):
        handle = self._files.pop(serial, None)
        if handle is not None:
            handle.close()

    def close(self):
        for serial in tuple(self._files):
            self.release(serial)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="tianji_cameras.preflight",
        description="Validate that every configured RealSense can produce the "
                    "schema-v1 RGB mode. Opens no pipeline and touches no device state.")
    parser.add_argument("--config", default=None,
                        help="collection config to read roles/serials from")
    args = parser.parse_args(argv)

    print("Configured cameras:")
    try:
        selection = load_camera_selection(args.config)
    except Exception as error:  # noqa: BLE001 - reported, never swallowed
        print(f"FAIL config: {error}", file=sys.stderr)
        return 2
    for role in CAMERA_ROLES:
        serial = selection.get(role)
        print(f"   {role}: {serial if serial else '(disabled)'}")

    print("Validating attached devices and stream modes:")
    try:
        profiles = validate_devices(selection)
    except PreflightError as error:
        print(f"FAIL {error}", file=sys.stderr)
        return 1
    print(f"\n{len(profiles)} camera(s) verified. No pipeline was opened.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
