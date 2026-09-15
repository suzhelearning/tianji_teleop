#!/usr/bin/env python3
"""Discover Wuji Hand 2 serials without enabling or commanding motors.

Default: USB/UDP discovery only. --read-handedness additionally connects each
Hand2 for one read-only handedness GET and disconnects. No clear-error, enable,
disable, calibration, target publisher or motion function is bound or called.
"""
from __future__ import annotations

import argparse
import ctypes as ct
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys


DEFAULT_LIBRARY = Path(__file__).resolve().parent / "vendor/wuji-sdk/lib/libwuji_sdk_c.so"


class Discovered(ct.Structure):
    # vendor/wuji-sdk/include/wuji_sdk.h: WujiDiscovered; enums are uint8_t.
    _fields_ = [("serial_number", ct.c_char * 64), ("model", ct.c_char * 32),
                ("device_id", ct.c_uint8), ("transport", ct.c_uint8),
                ("address", ct.c_char * 64)]


class ConnectTarget(ct.Structure):
    _fields_ = [("kind", ct.c_uint8), ("value", ct.c_char_p)]


class ConnectOptions(ct.Structure):
    _fields_ = [("timeout_ms", ct.c_uint32), ("retry_count", ct.c_uint32),
                ("enable_bridge", ct.c_bool), ("auto_time_sync_interval_ms", ct.c_uint64),
                ("auto_time_sync_interval_enabled", ct.c_bool)]


def load_sdk(path: Path):
    sdk = ct.CDLL(str(path.resolve()))
    signatures = {
        "wuji_init": ([ct.c_void_p], ct.c_int32),
        "wuji_shutdown": ([], None),
        "wuji_version": ([], ct.c_char_p),
        "wuji_last_error": ([], ct.c_char_p),
        "wuji_scan": ([ct.POINTER(ct.POINTER(Discovered)), ct.POINTER(ct.c_size_t)], ct.c_int32),
        "wuji_discovered_free": ([ct.POINTER(Discovered), ct.c_size_t], None),
        "wuji_connect_options_default": ([], ConnectOptions),
        "wuji_connect": ([ct.POINTER(ConnectTarget), ct.c_char_p,
                           ct.POINTER(ConnectOptions), ct.POINTER(ct.c_void_p)], ct.c_int32),
        "wuji_hand_2_get_handedness": ([ct.c_void_p, ct.POINTER(ct.c_int)], ct.c_int32),
        "wuji_dev_disconnect": ([ct.c_void_p], ct.c_int32),
        "wuji_dev_release": ([ct.c_void_p], None),
    }
    for name, (arguments, result) in signatures.items():
        function = getattr(sdk, name)
        function.argtypes, function.restype = arguments, result
    return sdk


def check(sdk, status: int, operation: str) -> None:
    if status != 0:
        error = sdk.wuji_last_error()
        detail = error.decode("utf-8", errors="replace") if error else "no SDK detail"
        raise RuntimeError(f"{operation} failed ({status}): {detail}")


def read_side(sdk, serial_number: str, timeout_ms: int) -> str:
    if not serial_number:
        raise RuntimeError("cannot query handedness without an unambiguous serial number")
    target = ConnectTarget(0, serial_number.encode("utf-8"))
    options = sdk.wuji_connect_options_default()
    options.timeout_ms = timeout_ms
    options.retry_count = 0
    options.auto_time_sync_interval_enabled = False
    handle = ct.c_void_p()
    check(sdk, sdk.wuji_connect(ct.byref(target), b"hand_identity_reader",
                                ct.byref(options), ct.byref(handle)), "connect for identity read")
    if not handle.value:
        raise RuntimeError("SDK returned a null identity-query handle")
    try:
        side = ct.c_int(-1)
        check(sdk, sdk.wuji_hand_2_get_handedness(handle, ct.byref(side)), "read handedness")
        if side.value not in (0, 1):
            raise RuntimeError(f"device reported unknown handedness value {side.value}")
        return ("left", "right")[side.value]
    finally:
        try:
            check(sdk, sdk.wuji_dev_disconnect(handle), "disconnect identity query")
        finally:
            sdk.wuji_dev_release(handle)


def discover(library: Path, query_handedness: bool = False, timeout_ms: int = 1000) -> dict:
    sdk = load_sdk(library)
    check(sdk, sdk.wuji_init(None), "initialize discovery SDK")
    devices = []
    try:
        records = ct.POINTER(Discovered)()
        count = ct.c_size_t()
        check(sdk, sdk.wuji_scan(ct.byref(records), ct.byref(count)), "scan USB/UDP")
        try:
            if count.value and not records:
                raise RuntimeError("SDK returned a null discovery array with a nonzero count")
            for index in range(count.value):
                raw = records[index]
                if raw.device_id != 2:  # WUJI_DEVICE_TYPE_WUJI_HAND_2
                    continue
                devices.append({
                    "serial_number": raw.serial_number.decode("utf-8", errors="replace"),
                    "model": raw.model.decode("utf-8", errors="replace"),
                    "address": raw.address.decode("utf-8", errors="replace"),
                    "transport": {0: "udp", 1: "usb", 2: "zenoh"}.get(raw.transport, "unknown"),
                    "side": "unknown",
                    "query_error": None,
                })
        finally:
            sdk.wuji_discovered_free(records, count.value)
        devices.sort(key=lambda device: (device["serial_number"], device["address"]))
        if query_handedness:
            for device in devices:
                try:
                    device["side"] = read_side(sdk, device["serial_number"], timeout_ms)
                except RuntimeError as error:
                    device["query_error"] = str(error)
        return {
            "scanned_at": datetime.now(timezone.utc).isoformat(),
            "sdk_version": sdk.wuji_version().decode("utf-8", errors="replace"),
            "handedness_queried": query_handedness,
            "devices": devices,
        }
    finally:
        sdk.wuji_shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-library", type=Path, default=DEFAULT_LIBRARY,
                        help="pinned Wuji C SDK library (default: local vendor directory)")
    parser.add_argument("--read-handedness", action="store_true",
                        help="connect only to GET left/right identity, then disconnect; no motor commands")
    parser.add_argument("--timeout-ms", type=int, default=1000, help="per-device identity query timeout")
    parser.add_argument("--json", action="store_true", help="print a machine-readable JSON report")
    parser.add_argument("--output", type=Path, help="save JSON to a NEW file; never overwrite configuration")
    args = parser.parse_args(argv)
    if args.timeout_ms <= 0:
        parser.error("--timeout-ms must be positive")
    if args.output and args.output.exists():
        parser.error(f"refusing to overwrite {args.output}")

    # SDK native/background logs use fd 1. Keep them on stderr so --json remains
    # parseable, while retaining the caller's original stdout for the report.
    with os.fdopen(os.dup(sys.stdout.fileno()), "w", encoding="utf-8") as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        try:
            report = discover(args.sdk_library, args.read_handedness, args.timeout_ms)
            serialized = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
            if args.output:
                with args.output.open("x", encoding="utf-8") as saved:
                    saved.write(serialized)
            if args.json:
                output.write(serialized)
            else:
                print(f"SDK {report['sdk_version']} | Hand2 devices: {len(report['devices'])}", file=output)
                print(f"{'SIDE':<8} {'SERIAL NUMBER':<26} {'TRANSPORT':<10} ADDRESS", file=output)
                for device in report["devices"]:
                    print(f"{device['side']:<8} {device['serial_number']:<26} "
                          f"{device['transport']:<10} {device['address']}", file=output)
                    if device["query_error"]:
                        print(f"  query error: {device['query_error']}", file=output)
                if not report["devices"]:
                    print("No Hand2 discovered. Check power, USB/network and host subnet.", file=output)
                elif not args.read_handedness:
                    print("Discovery does not include left/right. Use --read-handedness to query it; do not infer from row order.", file=output)
                if args.output:
                    print(f"Saved: {args.output}", file=output)
            return 0
        except (OSError, AttributeError, RuntimeError) as error:
            print(f"Hand2 discovery error: {error}", file=sys.stderr)
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
