"""Operator-owned recording episodes; no SDK calls and no blocking RPC waits.

One explicit enable covers the collection run. Each episode aligns from Home,
records only live teleoperation, stops following, saves/discards, and returns
Home without disabling. Recording service clients never grant motion authority.
"""
from __future__ import annotations

from .safety import SafetyFault


class CollectionEpisodes:
    def __init__(self, gate, observer, *, timeout_s=300.0, notify=print):
        self.gate = gate
        self.observer = observer
        self.state = "WAITING"
        self.done = False
        self._notify = notify
        self._timeout_ns = int(timeout_s * 1e9)
        self._key = None
        self._finish = False
        self._future = None
        self._requested_ns = 0
        self._deadline_ns = 0
        self._operation = None
        self._previous_saved = ""
        self._active_path = ""
        self._completed = False
        self._number = 0
        self._episode_id = ""
        self._saved_path = ""

    def on_key(self, key):
        """Accept only a current-state action; never queue the next episode."""
        allowed = {
            "WAITING": ("q",),
            "HOME_READY": ("r", "q"),
            "RECORDING": ("s", "d", "q"),
        }
        if key not in allowed.get(self.state, ()) or self._key is not None:
            self._notify(f"EPISODE {self.state}: {key} ignored; no queued/replayed action")
            return
        self._key = key

    def enabled(self):
        self._set_state("INITIAL_HOME")

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self._notify(f"EPISODE {state} | completed={self._number}")

    def _status(self):
        status = self.observer.status()
        if status is None:
            raise SafetyFault("collection episode status is missing or stale")
        if status.error or status.state in ("FAILED", "CLOSED", "ABORTING"):
            raise SafetyFault(f"collection episode failed: {status.error or status.state}")
        if status.session_id not in ("", self.observer.session_id):
            raise SafetyFault("collection episode belongs to a different executor session")
        return status

    def _submit(self, operation, status, now_ns):
        self._operation = operation
        self._requested_ns = now_ns
        self._deadline_ns = now_ns + self._timeout_ns
        self._previous_saved = status.last_saved_path
        self._active_path = status.active_path
        self._completed = False
        if operation == "start":
            self._episode_id = ""
        self._saved_path = ""
        self._future = self.observer.submit_recording(operation)

    def _acknowledged(self, now_ns):
        if now_ns > self._deadline_ns:
            raise SafetyFault(f"collection {self._operation} outcome timed out; no automatic retry")
        if not self._future.done():
            return False
        try:
            reply = self._future.result()
        except Exception as error:
            raise SafetyFault(f"collection {self._operation} failed: {error}") from error
        if not reply.success:
            raise SafetyFault(f"collection {self._operation} rejected: {reply.message}")
        if reply.session_id != self.observer.session_id or not reply.episode_id:
            raise SafetyFault("collection result identity mismatch")
        if self._operation == "start":
            if reply.state != "RECORDING":
                raise SafetyFault("collector has not completed recording start")
            self._episode_id = reply.episode_id
        elif reply.episode_id != self._episode_id or reply.state != "IDLE":
            raise SafetyFault("collector has not completed this episode")
        self._saved_path = reply.saved_path
        return True

    def _current_result(self, status):
        return (status.session_id == self.observer.session_id
                and status.published_monotonic_ns >= self._requested_ns)

    def tick(self, frame, feedback, now_ns):
        """Called before the guarded motion step. Never waits for disk or DDS."""
        key, self._key = self._key, None
        if self.state == "WAITING":
            if key == "q":
                self.done = True
                self._set_state("DONE")
            return

        status = self._status()
        if self.state in ("INITIAL_HOME", "HOMING"):
            if self.gate.phase != "HOME_REACHED":
                return
            if status.state != "IDLE":
                raise SafetyFault("collector is not idle at Home; next episode refused")
            if self.state == "HOMING":
                self._number += 1
            if self._finish:
                self.done = True
                self._set_state("DONE")
            else:
                self._set_state("HOME_READY")
                self._notify("HOME_READY | enabled, no following | r: next episode | q: finish and disable")
            return

        if self.state == "HOME_READY":
            if self.gate.phase != "HOME_REACHED":
                raise SafetyFault("episode Home hold was lost")
            if key == "q":
                self.done = True
                self._set_state("DONE")
            elif key == "r":
                if status.state != "IDLE" or not status.prepared or not status.inputs_ready:
                    self._notify("EPISODE start refused: collector must be idle with fresh inputs")
                    return
                self.gate.align_episode(frame, feedback, now_ns)
                self._set_state("ALIGNING")
            return

        if self.state == "ALIGNING":
            if self.gate.phase == "READY":
                self.gate.start_teleop(frame, feedback, now_ns)
                # Present a real, enabled TELEOP phase to the collector, but
                # retain the verified alignment hold until RECORDING is observed.
                self.gate.hold_teleop()
                self.observer.update_state(self.gate.phase)
                self._submit("start", status, now_ns)
                self._set_state("STARTING")
            return

        if self.state == "STARTING":
            if self._acknowledged(now_ns) and self._current_result(status):
                if (status.state == "RECORDING" and status.active_path
                        and status.episode_id == self._episode_id):
                    if self.gate.teleop_stopped:
                        self.gate.release_teleop_hold()
                        self._active_path = status.active_path
                        self._set_state("RECORDING")
                elif status.state not in ("IDLE", "STARTING", "RECORDING"):
                    raise SafetyFault(f"unexpected collector start state {status.state}")
            return

        if self.state == "RECORDING":
            if (status.state != "RECORDING" or status.active_path != self._active_path
                    or status.session_id != self.observer.session_id
                    or status.episode_id != self._episode_id):
                raise SafetyFault("active recording lost; stopping episode without automatic Home")
            if key in ("s", "d", "q"):
                self._finish = key == "q"
                self.gate.hold_teleop()
                self._submit("discard" if key == "d" else "save", status, now_ns)
                self._set_state("ENDING")
            return

        if self.state == "ENDING":
            acknowledged = self._acknowledged(now_ns)
            if acknowledged and self._current_result(status):
                if (status.state == "IDLE" and not status.active_path
                        and status.last_episode_id == self._episode_id):
                    if self._operation == "save" and (
                            not self._saved_path or status.last_saved_path != self._saved_path
                            or status.last_saved_path == self._previous_saved):
                        raise SafetyFault("save acknowledged but no new saved episode was confirmed")
                    self._completed = True
                elif status.state not in ("IDLE", "RECORDING", "SAVING", "DISCARDING"):
                    raise SafetyFault(f"unexpected collector finish state {status.state}")
            if self._completed and self.gate.teleop_stopped:
                self.gate.start_homing(frame, feedback, now_ns)
                self._set_state("HOMING")
            return

        if self.state != "DONE":
            raise SafetyFault(f"unknown collection episode state {self.state}")
