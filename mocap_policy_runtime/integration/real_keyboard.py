"""Terminal commands and physical Enter state; repeat bytes are never a deadman."""
from __future__ import annotations

import ctypes
import fcntl
import glob
import os
import select
import struct
import sys
import termios
import tty

from real_robot.safety import SafetyFault


class X11KeyState:
    """Source-compatible physical Return/KP_Enter query, including Wayland evdev."""

    _EVENT = struct.Struct("@llHHi")

    def __init__(self):
        self._fds = []
        self._buffers = {}
        self._pressed = {}
        self._display = None
        if os.environ.get("XDG_SESSION_TYPE") == "wayland":
            paths = sorted(glob.glob("/dev/input/by-id/*-event-kbd"))
            if not paths:
                raise SafetyFault("--hold-enter requires readable physical keyboard devices")
            try:
                for path in paths:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                    self._fds.append(fd)
                    self._buffers[fd] = b""
                    # EVIOCGKEY: event streams do not report keys already held
                    # when opened. Seed both Enter keys before accepting edges.
                    keymap = bytearray(96)
                    fcntl.ioctl(fd, (2 << 30) | (len(keymap) << 16) | (ord("E") << 8) | 0x18, keymap)
                    for code in (28, 96):
                        self._pressed[fd, code] = bool(keymap[code >> 3] & (1 << (code & 7)))
            except OSError:
                self.close()
                raise
            return
        display = os.environ.get("DISPLAY")
        if not display:
            raise SafetyFault("--hold-enter requires X11 DISPLAY or Wayland keyboard access")
        x11 = ctypes.CDLL("libX11.so.6")
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.restype = ctypes.c_int
        x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        x11.XStringToKeysym.restype = ctypes.c_ulong
        x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char)]
        x11.XQueryKeymap.restype = ctypes.c_int
        self._x11 = x11
        self._display = x11.XOpenDisplay(display.encode())
        if not self._display:
            raise SafetyFault("cannot open X11 display for --hold-enter")
        self._codes = tuple(int(x11.XKeysymToKeycode(self._display, x11.XStringToKeysym(name)))
                            for name in (b"Return", b"KP_Enter"))
        if not all(self._codes):
            self.close()
            raise SafetyFault("cannot resolve physical Enter keys")
        self._keymap = ctypes.create_string_buffer(32)

    def is_pressed(self):
        if self._fds:
            for fd in self._fds:
                data = self._buffers[fd]
                while True:
                    try:
                        chunk = os.read(fd, 4096)
                    except BlockingIOError:
                        break
                    if not chunk:
                        raise SafetyFault("physical keyboard disconnected")
                    data += chunk
                complete = len(data) - len(data) % self._EVENT.size
                for offset in range(0, complete, self._EVENT.size):
                    _, _, event_type, code, value = self._EVENT.unpack_from(data, offset)
                    if event_type == 0 and code == 3:
                        raise SafetyFault("physical keyboard events lost")
                    if event_type == 1 and code in (28, 96):
                        self._pressed[fd, code] = value != 0
                self._buffers[fd] = data[complete:]
            return any(self._pressed.values())
        if self._display is None:
            raise SafetyFault("physical key query is closed")
        if not self._x11.XQueryKeymap(self._display, self._keymap):
            raise SafetyFault("physical key query failed")
        return any(self._keymap.raw[code >> 3] & (1 << (code & 7)) for code in self._codes)

    def close(self):
        for fd in self._fds:
            os.close(fd)
        self._fds.clear()
        if self._display is not None:
            self._x11.XCloseDisplay(self._display)
            self._display = None


class OperatorKeyboard:
    """Use target terminal cbreak convention, retaining signal-driven Ctrl+C."""

    def __init__(self, hold_enter=False):
        self._physical = X11KeyState() if hold_enter else None
        self._closed = False
        try:
            self._fd = sys.stdin.fileno()
            self._original = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd, termios.TCSANOW)
        except BaseException:
            if self._physical is not None:
                self._physical.close()
            raise

    def poll(self):
        keys = set()
        for _ in range(256):
            if not select.select([self._fd], [], [], 0)[0]:
                break
            value = os.read(self._fd, 1)
            if not value:
                raise SafetyFault("operator terminal closed")
            keys.add("enter" if value in (b"\n", b"\r") else value.decode("ascii", "ignore").lower())
        return keys, self._physical.is_pressed() if self._physical is not None else False

    def close(self):
        try:
            if not self._closed:
                termios.tcsetattr(self._fd, termios.TCSANOW, self._original)
                self._closed = True
        finally:
            if self._physical is not None:
                self._physical.close()
