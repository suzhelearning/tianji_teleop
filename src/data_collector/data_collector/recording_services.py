"""Request-scoped recording transactions; no device or motion authority.

All mutation is on the ROS executor thread. Writer transitions are delivered by
CollectorNode's guard condition; service coroutines never block that executor.
"""
from dataclasses import dataclass
import uuid

from rclpy.task import Future


@dataclass
class Operation:
    fingerprint: tuple
    future: Future
    response: object
    kind: str
    episode_id: str
    previous_saved: str = ""


class RecordingServices:
    MAX_REQUESTS = 4096

    def __init__(self, node):
        self.node = node
        self.operations = {}
        self.active_episode_id = ""
        self.active_session_id = ""
        self.last_episode_id = ""
        self.start_operation = None
        self.stop_operation = None

    def _reply(self, response, request, success, message, episode_id="", state=None, saved_path=""):
        response.success = success
        response.request_id = request.request_id
        response.session_id = request.session_id
        response.episode_id = episode_id
        response.state = self.node._session.state if state is None else state
        response.saved_path = saved_path
        response.message = message
        return response

    def submit(self, kind, request, response):
        """Return one shared future for identical in-flight or completed requests."""
        fields = (request.session_id, request.phase_revision)
        fields += ((request.task,) if kind == "start" else
                   (request.episode_id, request.save, request.abort, request.cutoff_monotonic_ns))
        fingerprint = (kind, *fields)
        episode = request.request_id if kind == "start" else request.episode_id
        future = Future()
        try:
            if str(uuid.UUID(request.request_id)) != request.request_id:
                raise ValueError("request_id must be a canonical UUID")
        except (ValueError, AttributeError):
            future.set_result(self._reply(response, request, False, "request_id must be a canonical UUID"))
            return future
        previous = self.operations.get(request.request_id)
        if previous is not None:
            if previous.fingerprint == fingerprint:
                return previous.future
            future.set_result(self._reply(response, request, False, "request_id reused with different payload", episode))
            return future
        operation = Operation(fingerprint, future, response, kind, episode)
        # Never evict an accepted ID and then accidentally execute it again.
        # Capacity stops new admission, but cannot prevent closing the active file.
        if len(self.operations) >= self.MAX_REQUESTS and not (
                kind == "stop" and episode == self.active_episode_id and
                request.session_id == self.active_session_id):
            future.set_result(self._reply(response, request, False, "request history full; restart idle collector", episode))
            return future
        self.operations[request.request_id] = operation
        self._reply(response, request, False, "pending", episode)
        try:
            if self.node._stopping:
                raise ValueError("collector is stopping")
            if kind == "start":
                self._start(operation, request)
            else:
                self._stop(operation, request)
        except (ValueError, RuntimeError) as error:
            self._complete(operation, False, str(error))
        return future

    def _complete(self, operation, success, message, *, state=None, saved_path=""):
        if operation is None or operation.future.done():
            return
        response = operation.response
        response.success = success
        response.message = message
        response.state = self.node._session.state if state is None else state
        response.saved_path = saved_path
        operation.future.set_result(response)

    def _authorize(self, kind, request):
        allowed, reason = self.node._authorize(kind, request)
        if not allowed:
            raise ValueError(reason)

    def _start(self, operation, request):
        session = self.node._session
        self._authorize("start", request)
        if request.task and request.task != session.task:
            raise ValueError("task does not match collector configuration")
        if self.active_episode_id or session.state != "IDLE":
            raise ValueError("collector already owns an episode or is not idle")
        ready, detail = session.check_ready()
        if not ready:
            raise ValueError(detail)
        self.active_episode_id = operation.episode_id
        self.active_session_id = request.session_id
        self.start_operation = operation
        self.stop_operation = None
        self.node._recording_executor = (request.session_id, request.phase_revision)
        session.command("r", "TELEOP")

    def _stop(self, operation, request):
        session = self.node._session
        if request.save and request.abort:
            raise ValueError("save and abort are mutually exclusive")
        if (not request.episode_id or request.episode_id != self.active_episode_id
                or request.session_id != self.active_session_id):
            raise ValueError("no active recording for this executor session and episode")
        if self.stop_operation is not None:
            raise ValueError("an end operation already owns this episode")
        if request.cutoff_monotonic_ns and session.state != "RECORDING":
            raise ValueError("cutoff requires an active recording")
        if request.abort:
            if session.state not in ("STARTING", "RECORDING", "ABORTING"):
                raise ValueError("episode cannot be aborted in this state")
        else:
            self._authorize("stop", request)
            if session.state != "RECORDING":
                raise ValueError("episode is not recording")
        operation.kind = "abort" if request.abort else ("save" if request.save else "discard")
        operation.previous_saved = str(session.saved_paths[-1]) if session.saved_paths else ""
        if request.abort:
            session.end_episode(request.cutoff_monotonic_ns)
        else:
            session.command("s" if request.save else "d", "TELEOP", request.cutoff_monotonic_ns)
        self.stop_operation = operation

    def transition(self, state, active_path, saved_path, operation_error):
        """Complete only the episode whose serialized writer transitions we own."""
        if not self.active_episode_id:
            return
        if state == "RECORDING" and self.stop_operation is None:
            self._complete(self.start_operation, bool(active_path),
                           "recording started" if active_path else "recording has no writer", state=state)
        if state not in ("IDLE", "FAILED", "CLOSED"):
            return
        self._complete(self.start_operation, False, operation_error or "start ended before recording", state=state)
        ending = self.stop_operation
        if ending is not None:
            success = state == "IDLE" and not active_path and not operation_error
            if ending.kind == "save":
                success = success and bool(saved_path) and saved_path != ending.previous_saved
            self._complete(ending, success,
                           f"{ending.kind} completed" if success else (operation_error or "episode finalization failed"),
                           state=state, saved_path=saved_path if success and ending.kind == "save" else "")
        if active_path:
            # A failed close can leave the writer owned. Do not advertise that
            # episode as finalized or allow its identity to be replaced.
            return
        self.last_episode_id = self.active_episode_id
        self.active_episode_id = self.active_session_id = ""
        self.start_operation = self.stop_operation = None
        self.node._recording_executor = None

    def close(self):
        for operation in self.operations.values():
            self._complete(operation, False, "collector shutdown; unfinished result", state="CLOSED")
