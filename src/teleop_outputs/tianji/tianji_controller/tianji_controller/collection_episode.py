"""Frozen-target approach, confirmed recording, and immediate stop-key takeover.

Recording and movement have independent lifetimes. An s/d key closes data at
its receipt timestamp and starts bounded braking immediately. Home begins only
after measured rest and the recording result. No RPC alone grants motor authority.
"""
from __future__ import annotations

import time

from .safety import SafetyFault


class CollectionEpisodes:
    def __init__(self, gate, observer, *, timeout_s=300.0,
                 notify=print, clock=time.monotonic_ns):
        self.gate = gate
        self.observer = observer
        self.state = "WAITING"
        self.done = False
        self._notify = notify
        self._clock = clock
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
        self._notice = ""
        self._retry_start = False

    @property
    def detail(self):
        if self.state == "WAITING":
            return "确认机器人双手无物体｜r：使能并缓慢张开双手"
        if self.state in ("INITIAL_HOME", "HOMING"):
            if self.gate.opening_hands:
                return "正在张开双手｜请等待，尚未跟随"
            return "正在准备双手｜机械臂保持 Home" if self.state == "INITIAL_HOME" else "正在回 Home｜请等待，不再录制"
        if self.state == "HOME_READY":
            return self._notice or "已就绪｜预览实时目标｜踩 r 冻结目标并缓慢对齐"
        if self.state == "ALIGNING":
            return "正在缓慢对齐冻结目标｜尚未录制，不跟随实时动作"
        if self.state == "ALIGNED":
            return self._notice or "已对齐｜正在确认静止，尚未录制"
        if self.state == "STARTING":
            return "正在确认录制｜保持冻结目标，尚未跟随"
        if self.state == "RECORDING":
            return "录制中｜正在跟随｜s 保存 / d 丢弃"
        if self.state == "STOPPING":
            return "数据已截止｜正在停止跟随，请等待安全回 Home"
        if self.state == "DONE":
            return "本场已结束｜机器人已完成准备，将退出使能"
        return "控制状态未知｜不要开始操作"

    def on_key(self, key):
        """The key timestamp, not an eventual RPC response, defines data cutoff."""
        allowed = {
            "WAITING": ("r", "q"), "HOME_READY": ("r", "q"), "ALIGNED": ("r",),
            "RECORDING": ("s", "d", "q"), "STOPPING": ("q",),
        }
        if key not in allowed.get(self.state, ()) or self._key is not None:
            self._notify(f"EPISODE {self.state}: {key} ignored; no queued/replayed action")
            return
        self._key = (key, self._clock())

    def enabled(self):
        self._set_state("INITIAL_HOME")

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self._notify(f"EPISODE {state} | completed={self._number} | {self.detail}")

    def _status(self):
        status = self.observer.status()
        if status is None:
            raise SafetyFault("collection episode status is missing or stale")
        if status.error or status.state in ("FAILED", "CLOSED", "ABORTING"):
            raise SafetyFault(f"collection episode failed: {status.error or status.state}")
        if status.session_id not in ("", self.observer.session_id):
            raise SafetyFault("collection episode belongs to a different executor session")
        return status

    def _submit(self, operation, status, now_ns, *, cutoff_ns=0):
        self._operation = operation
        self._requested_ns = now_ns
        self._deadline_ns = now_ns + self._timeout_ns
        self._previous_saved = status.last_saved_path
        self._active_path = status.active_path
        self._completed = False
        if operation == "start":
            self._episode_id = ""
        self._saved_path = ""
        self._future = self.observer.submit_recording(operation, cutoff_monotonic_ns=cutoff_ns)

    def _reply(self, now_ns):
        if now_ns > self._deadline_ns:
            raise SafetyFault(f"collection {self._operation} outcome timed out; no automatic retry")
        if not self._future.done():
            return None
        try:
            reply = self._future.result()
        except Exception as error:
            raise SafetyFault(f"collection {self._operation} failed: {error}") from error
        if reply.session_id != self.observer.session_id or not reply.episode_id:
            raise SafetyFault("collection result identity mismatch")
        if self._operation == "start":
            self._episode_id = reply.episode_id
        elif reply.episode_id != self._episode_id:
            raise SafetyFault("collector completed a different episode")
        if reply.success and reply.state != ("RECORDING" if self._operation == "start" else "IDLE"):
            raise SafetyFault("collector has not completed the requested operation")
        return reply

    def _current_result(self, status):
        return (status.session_id == self.observer.session_id
                and status.published_monotonic_ns >= self._requested_ns)

    def _finish_recording(self, status, now_ns):
        if self._completed:
            return
        reply = self._reply(now_ns)
        if reply is None:
            return
        if not reply.success:
            raise SafetyFault(f"collection {self._operation} rejected: {reply.message}")
        if not self._current_result(status):
            return
        if status.state == "IDLE" and not status.active_path and status.last_episode_id == self._episode_id:
            if self._operation == "save" and (
                    not reply.saved_path or status.last_saved_path != reply.saved_path
                    or status.last_saved_path == self._previous_saved):
                raise SafetyFault("save acknowledged but no new saved episode was confirmed")
            self._saved_path = reply.saved_path
            self._completed = True
            self._notify("数据已保存：" + reply.saved_path if self._operation == "save" else "本条数据已丢弃")
        elif status.state not in ("IDLE", "RECORDING", "SAVING", "DISCARDING"):
            raise SafetyFault(f"unexpected collector finish state {status.state}")

    def tick(self, frame, feedback, now_ns):
        """One nonblocking step before guarded output; no SDK, disk or RPC wait."""
        action, self._key = self._key, None
        key, key_ns = action if action is not None else (None, 0)
        if self.state == "WAITING":
            if key == "q":
                self.done = True
                self._set_state("DONE")
            return key == "r"
        status = self._status()
        if self.state in ("INITIAL_HOME", "HOMING"):
            if self.gate.phase != "HOME_REACHED":
                return
            if status.state != "IDLE" or status.active_path:
                raise SafetyFault("collector is not idle at Home; next episode refused")
            if self.state == "HOMING":
                self._number += 1
            self._notice = ""
            if self._finish:
                self.done = True
                self._set_state("DONE")
            else:
                self._set_state("HOME_READY")
            return
        if self.state == "HOME_READY":
            if self.gate.phase != "HOME_REACHED":
                raise SafetyFault("episode Home hold was lost")
            if key == "q":
                self.done = True
                self._set_state("DONE")
            elif key == "r":
                if status.state != "IDLE" or status.active_path or not status.prepared or not status.inputs_ready:
                    self._notice = "未开始｜采集器或相机尚未就绪，请就绪后重新踩 r"
                    self._notify(self._notice)
                    return
                try:
                    self.gate.start_collection_alignment(frame, feedback, now_ns)
                except ValueError as error:
                    self._notice = f"未开始｜机器人尚未在 Home 静止并张开双手，请重新踩 r\n{error}"
                    self._notify(self._notice)
                    return
                self._notice = ""
                self._retry_start = False
                self._set_state("ALIGNING")
            return
        if self.state == "ALIGNING":
            if status.state != "IDLE" or status.active_path:
                raise SafetyFault("collector became active during unrecorded alignment")
            if self.gate.phase == "READY":
                self.gate.start_teleop(frame, feedback, now_ns)
                self.gate.hold_teleop()
                self._set_state("ALIGNED")
            return
        if self.state == "ALIGNED":
            if status.state != "IDLE" or status.active_path:
                raise SafetyFault("collector is not idle before recording start")
            if not self.gate.teleop_stopped or (self._retry_start and key != "r"):
                return
            if not status.prepared or not status.inputs_ready:
                self._retry_start = True
                self._notice = "已对齐并保持静止｜采集器或相机尚未就绪，请就绪后重新踩 r"
                self._notify(self._notice)
                return
            self._notice = ""
            self._set_state("STARTING")
            self.observer.update_state(self.gate.phase, detail=self.detail)
            self._submit("start", status, now_ns)
            return
        if self.state == "STARTING":
            reply = self._reply(now_ns)
            if reply is None:
                return
            if not reply.success:
                if reply.state != "IDLE" or status.state != "IDLE" or status.active_path:
                    raise SafetyFault(f"collection start rejected with uncertain state: {reply.message}")
                if not self._current_result(status):
                    return
                self._retry_start = True
                self._notice = "采集未启动｜保持静止，请检查提示后重新踩 r"
                self._set_state("ALIGNED")
                return
            if not self._current_result(status):
                return
            if status.state == "RECORDING" and status.active_path and status.episode_id == self._episode_id:
                if not self.gate.teleop_stopped:
                    return
                self._active_path = status.active_path
                self.gate.release_teleop_hold()
                self._set_state("RECORDING")
            elif status.state not in ("IDLE", "STARTING", "RECORDING"):
                raise SafetyFault(f"unexpected collector start state {status.state}")
            return
        if self.state == "RECORDING":
            if (status.state != "RECORDING" or status.active_path != self._active_path
                    or status.session_id != self.observer.session_id or status.episode_id != self._episode_id):
                raise SafetyFault("active recording lost; stopping without automatic Home")
            if key in ("s", "d", "q"):
                self._finish = key == "q"
                self._submit("discard" if key == "d" else "save", status, now_ns, cutoff_ns=key_ns)
                self.gate.hold_teleop()
                self._set_state("STOPPING")
            return
        if self.state == "STOPPING":
            if key == "q":
                self._finish = True
            self._finish_recording(status, now_ns)
            if self._completed and self.gate.teleop_stopped:
                self.gate.start_homing(frame, feedback, now_ns)
                self._set_state("HOMING")
            return
        if self.state != "DONE":
            raise SafetyFault(f"unknown collection episode state {self.state}")
