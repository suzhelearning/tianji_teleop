"""Single-key operator keyboard shared by the executor and the collector.

Keeps the terminal in cbreak mode with ``ISIG`` intact, so Ctrl+C stays the
normal executor stop signal. The handler knows nothing about robot safety: it
reports raw keys and the caller decides what they mean.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty


class OperatorKeyboard:
    """Read single keys without blocking; Enter is reported separately."""

    def __init__(self, on_key, on_calibrate=None, *, on_finish=None):
        self._on_key = on_key
        self._on_calibrate = on_calibrate
        self._on_finish = on_finish
        self._fd = sys.stdin.fileno()
        self._original = termios.tcgetattr(self._fd)
        self._closed = False
        tty.setcbreak(self._fd, termios.TCSANOW)

    def poll_enter(self) -> bool:
        """Drain pending input; return True if Enter was pressed.

        Raises :class:`EOFError` when the operator terminal disappears, which
        callers must treat as a stop condition.
        """
        for _ in range(256):
            if not select.select([self._fd], [], [], 0)[0]:
                return False
            character = os.read(self._fd, 1)
            if not character:
                raise EOFError("operator terminal closed")
            if character in (b"\n", b"\r"):
                return True
            if character.lower() == b"c" and self._on_calibrate is not None:
                self._on_calibrate()
                continue
            if character.lower() == b"q" and self._on_finish is not None:
                self._on_finish()
                continue
            if character.lower() in (b"r", b"s", b"d"):
                try:
                    self._on_key(character.decode("ascii").lower())
                except Exception as error:  # noqa: BLE001 - never kill the control loop
                    print(f"KEY HANDLER FAILED (robot control unchanged): {error}", flush=True)
        return False

    def close(self) -> None:
        """Restore the terminal.

        Never raises: this runs on cleanup paths where the terminal may already
        be gone, and a failure to restore must not mask the real error that
        caused the shutdown.
        """
        if self._closed:
            return
        self._closed = True
        try:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._original)
        except (termios.error, OSError, ValueError):
            pass
