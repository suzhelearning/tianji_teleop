#!/usr/bin/env python3
"""Prepare the wired APK's device-to-host tracking connection on TCP 9999."""
import os
import subprocess
import sys


def ensure_adb_reverse():
    """Reuse or create the selected device's reverse rule; never replace an owner."""
    def adb(*arguments):
        try:
            return subprocess.run(
                ["adb", *arguments], check=True, capture_output=True,
                text=True, timeout=5).stdout
        except FileNotFoundError as error:
            raise RuntimeError("ADB is required for wired PICO tracking; install adb first") from error
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("ADB preflight timed out; check the USB connection") from error
        except subprocess.CalledProcessError as error:
            raise RuntimeError(f"ADB preflight failed: {error.stderr.strip()}") from error

    def mappings(*arguments):
        rows = [line.split() for line in adb(*arguments).splitlines() if line.strip()]
        if any(len(row) != 3 for row in rows):
            raise RuntimeError("Unexpected ADB mapping list; refusing to change tracking mappings")
        return rows

    devices = {}
    for line in adb("devices").splitlines():
        fields = line.split()
        if len(fields) >= 2 and not line.startswith("List of devices"):
            devices[fields[0]] = fields[1]
    serial = os.environ.get("ANDROID_SERIAL")
    if not serial:
        if len(devices) != 1:
            raise RuntimeError("Connect one PICO headset and authorize USB debugging; "
                               "with multiple devices set ANDROID_SERIAL explicitly")
        serial = next(iter(devices))
    if devices.get(serial) != "device":
        raise RuntimeError(f"PICO {serial} is not authorized/online; "
                           "accept USB debugging in the headset")

    endpoint = "tcp:9999"

    def reverse_owners():
        # Reverse listings identify the transport, not necessarily its device serial.
        return [row[1:] for row in mappings("-s", serial, "reverse", "--list")
                if row[1] == endpoint]

    expected_reverse = [[endpoint, endpoint]]
    reverse = reverse_owners()
    if reverse and reverse != expected_reverse:
        raise RuntimeError(f"ADB device {serial} {endpoint} already reverses to another port; "
                           "refusing to replace it")

    # A legacy forward occupies the PC listener port. Retire only this exact
    # selected-device rule; never remove another device's rule or another port.
    forwards = [row for row in mappings("forward", "--list") if row[1] == endpoint]
    if forwards:
        if forwards != [[serial, endpoint, endpoint]]:
            raise RuntimeError(f"ADB host {endpoint} already forwards to another device/port; "
                               "refusing to remove it")
        adb("-s", serial, "forward", "--remove", endpoint)
        if any(row[1] == endpoint for row in mappings("forward", "--list")):
            raise RuntimeError(f"ADB host {endpoint} is still forwarded; refusing to start the listener")
        print(f"ADB retired legacy forward: {serial} host {endpoint} -> device {endpoint}",
              flush=True)

    if not reverse:
        adb("-s", serial, "reverse", "--no-rebind", endpoint, endpoint)
    if reverse_owners() != expected_reverse:
        raise RuntimeError(f"ADB did not retain the requested {endpoint} reverse rule")
    state = "reused" if reverse else "created"
    print(f"ADB ready: {serial} device {endpoint} -> host {endpoint} ({state})", flush=True)


def main():
    try:
        ensure_adb_reverse()
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
