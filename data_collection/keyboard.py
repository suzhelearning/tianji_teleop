"""Single-key recording shortcuts while retaining Enter and terminal Ctrl+C."""
import os
import select
import sys
import termios
import tty

from real_robot.safety import SafetyFault


class CollectionKeyboard:
    def __init__(self, on_key):
        self._fd = sys.stdin.fileno()
        self._original = termios.tcgetattr(self._fd)
        self._on_key = on_key
        self._closed = False
        # Keep ISIG, so Ctrl+C remains the executor's normal stop signal.
        tty.setcbreak(self._fd, termios.TCSANOW)

    def poll_enter(self):
        for _ in range(256):
            if not select.select([self._fd], [], [], 0)[0]:
                return False
            character = os.read(self._fd, 1)
            if not character:
                raise SafetyFault('operator terminal closed')
            if character in (b'\n', b'\r'):
                return True
            if character.lower() in (b'r', b's', b'd'):
                try:
                    self._on_key(character.decode('ascii').lower())
                except Exception as error:
                    print(f'DATASET KEY FAILED (robot control unchanged): {error}', flush=True)
        return False

    def close(self):
        if not self._closed:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._original)
            self._closed = True
