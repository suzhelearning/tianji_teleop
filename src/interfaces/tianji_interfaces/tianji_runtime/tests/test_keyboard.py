"""Terminal handling of the shared operator keyboard.

Uses a real pty because the behaviour under test is the terminal mode itself:
single keys must arrive without Enter, and the original terminal settings must be
restored on close so a crashed session cannot leave the operator's shell in
cbreak mode. Ctrl+C must stay a signal, which is why ISIG is preserved.
"""

from __future__ import annotations

import os
import pty
import sys
import termios
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tianji_runtime import OperatorKeyboard  # noqa: E402


def _open_pty():
    master, slave = pty.openpty()
    return master, slave, termios.tcgetattr(slave)


def test_single_keys_arrive_without_enter_and_enter_is_reported(monkeypatch):
    master, slave, original = _open_pty()
    keys: list[str] = []
    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(keys.append)
            try:
                os.write(master, b"rsd")
                assert keyboard.poll_enter() is False
                assert keys == ["r", "s", "d"]
                os.write(master, b"\n")
                assert keyboard.poll_enter() is True
            finally:
                keyboard.close()
                assert termios.tcgetattr(slave) == original, "terminal mode not restored"
    finally:
        os.close(master)
        os.close(slave)


def test_only_recording_keys_are_reported(monkeypatch):
    master, slave, _ = _open_pty()
    keys: list[str] = []
    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(keys.append)
            try:
                os.write(master, b"xyR\n")
                assert keyboard.poll_enter() is True
                # 'x'/'y' are not recording keys and must not reach the handler;
                # 'R' is normalised to lower case.
                assert keys == ["r"]
            finally:
                keyboard.close()
    finally:
        os.close(master)
        os.close(slave)


def test_calibrate_key_is_dispatched_separately(monkeypatch):
    master, slave, _ = _open_pty()
    keys: list[str] = []
    calibrations: list[int] = []
    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(keys.append, on_calibrate=lambda: calibrations.append(1))
            try:
                os.write(master, b"c")
                keyboard.poll_enter()
                assert calibrations == [1]
                assert keys == [], "'c' starts calibration, it is not a recording key"
            finally:
                keyboard.close()
    finally:
        os.close(master)
        os.close(slave)


def test_collection_finish_key_never_becomes_enter_authorization(monkeypatch):
    master, slave, _ = _open_pty()
    events = []
    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(events.append, on_finish=lambda: events.append("finish"))
            try:
                os.write(master, b"rQs")
                assert keyboard.poll_enter() is False
                assert events == ["r", "finish", "s"]
            finally:
                keyboard.close()
    finally:
        os.close(master)
        os.close(slave)


def test_key_handler_failure_does_not_break_polling(monkeypatch, capsys):
    master, slave, _ = _open_pty()
    handled: list[str] = []

    def handler(key):
        handled.append(key)
        if key == "s":
            raise RuntimeError("writer exploded")

    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(handler)
            try:
                # A failing handler must not stop the control loop from polling.
                os.write(master, b"srd\n")
                assert keyboard.poll_enter() is True
                assert handled == ["s", "r", "d"]
                assert "writer exploded" in capsys.readouterr().out
            finally:
                keyboard.close()
    finally:
        os.close(master)
        os.close(slave)


def test_closed_terminal_is_reported(monkeypatch):
    master, slave, _ = _open_pty()
    try:
        with os.fdopen(os.dup(slave), "r") as terminal:
            monkeypatch.setattr(sys, "stdin", terminal)
            keyboard = OperatorKeyboard(lambda key: None)
            os.close(master)
            master = None
            try:
                keyboard.poll_enter()
            except EOFError:
                pass
            finally:
                keyboard.close()
    finally:
        if master is not None:
            os.close(master)
        os.close(slave)
