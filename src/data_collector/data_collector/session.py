"""Independent R/S/D episode management; never changes the robot's TELEOP state.

The session owns episode lifecycle only. It holds no SDK handle, opens no
camera and never commands hardware: samples arrive from the collector node,
which forward them to the ROS acquisition runtime. That separation is what lets
a collector failure leave an episode partial without touching TELEOP.
"""
from pathlib import Path
import hashlib
import queue
import threading
import time

from .config import load_collection_config, dataset_metadata


class _WriterSlot:
    """Acquisition keeps one stable sink while episode files change in the background."""
    def __init__(self):
        self._lock = threading.Lock()
        self._writer = None

    def attach(self, writer):
        with self._lock:
            if self._writer is not None:
                raise RuntimeError('previous episode is still attached')
            self._writer = writer

    def current(self):
        with self._lock:
            return self._writer

    def detach(self):
        with self._lock:
            writer, self._writer = self._writer, None
            if writer is not None:
                writer.stop()
            return writer

    def start(self, start_ns):
        with self._lock:
            if self._writer is None:
                raise RuntimeError('episode writer is not ready')
            self._writer.start(start_ns)

    def stop(self):
        with self._lock:
            if self._writer is not None:
                self._writer.stop()

    def check(self):
        with self._lock:
            writer = self._writer
        if writer is not None:
            writer.check()

    def _append(self, method, *args):
        with self._lock:
            writer = self._writer
        return False if writer is None else getattr(writer, method)(*args)

    def append_arms(self, timestamp_ns, qpos):
        return self._append('append_arms', timestamp_ns, qpos)

    def append_hands(self, timestamp_ns, qpos):
        return self._append('append_hands', timestamp_ns, qpos)

    def append_rgb(self, name, timestamp_ns, rgb):
        return self._append('append_rgb', name, timestamp_ns, rgb)


class CollectionSession:
    def __init__(self, dataset_dir, task, config_path, model_path, *, camera_monitor=None,
                 on_transition=None):
        from .dataset import EpisodeWriter
        if not isinstance(task, str) or not task.strip():
            raise ValueError('collection requires a nonempty task label')
        config_bytes = Path(config_path).read_bytes()
        self.config, self.cameras = load_collection_config(config_path, source_bytes=config_bytes)
        self.config_digest = hashlib.sha256(config_bytes).hexdigest()
        model_bytes = Path(model_path).read_bytes()
        self.metadata = dataset_metadata(self.config, self.cameras, model_path)
        if Path(model_path).read_bytes() != model_bytes:
            raise RuntimeError('controller model changed while loading dataset metadata')
        self.model_digest = hashlib.sha256(model_bytes).hexdigest()
        # Only the ROS node's matched, continuously renewed monitor certificate
        # may establish camera readiness. Writer preflight alone is not prepared.
        self._camera_monitor = camera_monitor
        self.dataset_dir = Path(dataset_dir).resolve()
        self.task = task
        self._writer_factory = EpisodeWriter
        # Prove the directory/config/HDF5 are writable before connecting motors.
        # This unused prepared file is discarded if R is never pressed.
        self._prepared = EpisodeWriter(self.dataset_dir, self.metadata, self.task)
        self._sink = _WriterSlot()
        self._lock = threading.Lock()
        self._commands = queue.Queue(maxsize=8)
        self._on_transition = on_transition
        self._state = 'IDLE'
        self._closing = False
        self._finished = False
        self._last_error = None
        self._manager_error = None
        self._notice = None
        self._thread = None
        self.runtime = None
        self.saved_paths = []
        print(f'DATASET WRITER READY: {self.dataset_dir} task={task} cameras={self.cameras}; '
              'TELEOP keys: r=start, s=save success, d=discard current; no actions recorded', flush=True)

    def _set_state(self, state):
        """Called under _lock; handoff must never publish DDS or wait for a consumer."""
        if self._state == state:
            return
        self._state = state
        if self._on_transition is not None:
            writer = self._sink.current()
            self._on_transition(
                state, str(writer.partial_path) if writer is not None else '',
                str(self.saved_paths[-1]) if self.saved_paths else '',
                self.last_error or '')

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def last_error(self):
        """Latest acquisition or writer-manager failure for status subscribers."""
        return self._last_error or self._manager_error

    def current_writer(self):
        """The writer of the episode being recorded, or ``None``.

        Used by the status publisher to report the active partial path; the
        writer is attached and detached by the manager thread, so this is a
        snapshot rather than a stable handle.
        """
        return self._sink.current()

    @property
    def prepared(self):
        """Writer preflight succeeded; every episode still checks its own disk writes."""
        return not self._closing and self._manager_error is None

    def camera_ready(self):
        """``(ok, reason)`` for the configured camera streams.

        The node combines the standalone monitor's profile certificate with its
        own paired-stream validation; neither alone authorizes recording.
        """
        monitor = self._camera_monitor
        if monitor is None:
            return True, ''
        return monitor.status()

    def missing_inputs(self):
        """Inputs that are not currently usable, as operator-readable text."""
        if self.runtime is None:
            return ['measured feedback has not been received yet']
        return self.runtime.missing_inputs()

    def check_ready(self):
        """``(ok, reason)`` for the collector service; never opens a device."""
        problems = []
        ready, detail = self.camera_ready()
        if not ready:
            problems.append(f'cameras: {detail}')
        if not self.prepared:
            problems.append('writer is not prepared')
        problems.extend(self.missing_inputs())
        return (not problems, '; '.join(problems))

    def start(self, *, boot_id):
        """Attach the ROS acquisition runtime.

        ``boot_id`` is this host's boot identity; every incoming sample is
        checked against it so a sample from another machine can never be
        recorded as if it were local.
        """
        from .runtime import CollectionRuntime
        self.runtime = CollectionRuntime(self._sink, boot_id=boot_id, cameras=self.cameras)
        self.runtime.start()
        self._thread = threading.Thread(target=self._manage, name='episode-manager', daemon=True)
        self._thread.start()

    # -------------------------------------------------------- sample forwarding

    def on_arms(self, message, received_ns=None):
        if self.runtime is not None:
            self.runtime.on_arms(message, received_ns)

    def on_hand(self, side, message, received_ns=None):
        if self.runtime is not None:
            self.runtime.on_hand(side, message, received_ns)

    def on_image(self, role, image):
        if self.runtime is not None:
            self.runtime.on_image(role, image)

    def on_camera_fault(self, role, detail):
        if self.runtime is not None:
            self.runtime.on_camera_fault(role, detail)

    def _notify(self, text):
        if text != self._notice:
            print(text, flush=True)
            self._notice = text

    def command(self, key, phase):
        """Only enqueue work. R/S/D never call robot APIs, join threads or write files."""
        if phase != 'TELEOP':
            self._notify('DATASET: r/s/d are available only in TELEOP')
            return
        with self._lock:
            if self._closing:
                return
            if key == 'r' and self._state == 'IDLE':
                self._set_state('STARTING')
                self._commands.put_nowait('start')
            elif key in ('s', 'd') and self._state == 'RECORDING':
                # Freeze this segment immediately; saving/deletion remains asynchronous.
                self.runtime.end_episode()
                self._set_state('SAVING' if key == 's' else 'DISCARDING')
                self._commands.put_nowait('save' if key == 's' else 'discard')
            else:
                self._notify(f'DATASET {self._state}: key {key} ignored; saved episodes are never deleted by d')

    def check(self):
        """Report acquisition faults and preserve a partial segment, without stopping TELEOP."""
        if self.runtime is None:
            return
        try:
            self.runtime.check()
        except Exception as error:
            detail = str(error)
            if detail != self._last_error:
                self._last_error = detail
                self._notify(f'DATASET CAPTURE FAILED (robot control unchanged): {detail}')
            if self.state in ('STARTING', 'RECORDING'):
                self.end_episode()

    def end_episode(self):
        """An unsaved segment ends at HOME/stop/fault and is kept as partial, not success."""
        with self._lock:
            if self._state in ('STARTING', 'RECORDING'):
                if self.runtime is not None:
                    self.runtime.end_episode()
                self._set_state('ABORTING')
                self._commands.put_nowait('abort')

    def request_stop(self):
        self.end_episode()
        with self._lock:
            if not self._closing:
                self._closing = True
                self._commands.put_nowait('close')
        if self.runtime is not None:
            self.runtime.request_stop()

    def _manage(self):
        while True:
            operation = self._commands.get()
            try:
                if operation == 'close':
                    break
                if operation == 'start':
                    self._start_recording()
                else:
                    self._finish_recording(operation)
            except Exception as error:
                self._last_error = str(error)
                self._notify(f'DATASET OPERATION FAILED (robot control unchanged): {error}')
                cleanup_failed = False
                try:
                    if self.runtime is not None:
                        self.runtime.end_episode()
                    writer = self._sink.current()
                    if writer is not None:
                        writer.abort()
                        self._sink.detach()
                        self._notify(f'DATASET PARTIAL: {writer.partial_path}')
                except Exception as cleanup:
                    cleanup_failed = True
                    self._notify(f'DATASET CLEANUP FAILED: {cleanup}')
                with self._lock:
                    if cleanup_failed:
                        self._set_state('FAILED')
                    elif self._state != 'ABORTING':
                        self._set_state('IDLE')
            finally:
                self._commands.task_done()
        self._close_writers()

    def _close_writers(self):
        errors = []
        try:
            leftover = self._sink.current()
            if leftover is not None:
                leftover.abort()
                self._sink.detach()
        except Exception as error:
            errors.append(str(error))
        try:
            if self._prepared is not None:
                self._prepared.discard()
                self._prepared = None
        except Exception as error:
            errors.append(str(error))
        self._manager_error = '; '.join(errors) if errors else None
        if self._manager_error:
            self._notify(f'DATASET CLEANUP FAILED: {self._manager_error}')
        with self._lock:
            self._set_state('FAILED' if errors else 'CLOSED')

    def _start_recording(self):
        with self._lock:
            if self._state != 'STARTING' or self._closing:
                return
        ready, detail = self.camera_ready()
        if not ready:
            raise RuntimeError(f'camera certification revoked before recording: {detail}')
        writer = self._prepared
        self._prepared = None
        if writer is None:
            writer = self._writer_factory(self.dataset_dir, self.metadata, self.task)
        self._sink.attach(writer)
        with self._lock:
            permitted = self._state == 'STARTING' and not self._closing
        if not permitted:
            return  # the queued abort/close owns this file now
        self.runtime.begin_episode(time.monotonic_ns())
        with self._lock:
            if self._state == 'STARTING' and not self._closing:
                self._last_error = None
                self._set_state('RECORDING')
                self._notify(f'DATASET RECORDING: {writer.partial_path}; s=save, d=discard')
            else:
                self.runtime.end_episode()

    def _finish_recording(self, operation):
        self.runtime.end_episode()
        if operation == 'save':
            self.runtime.check_episode()
        # Keep ownership until closure is confirmed. A timed-out close is retried
        # by the manager's error/shutdown path, never abandoned after detach.
        writer = self._sink.current()
        if writer is not None:
            if operation == 'save':
                path = writer.finish(success=True)
                self.saved_paths.append(path)
                self._notify(f'DATASET SAVED: {path} success=true; TELEOP continues, r starts next episode')
            elif operation == 'discard':
                writer.discard()
                self._notify('DATASET DISCARDED: current segment only; TELEOP continues')
            else:
                writer.abort()
                self._notify(f'DATASET PARTIAL (not saved by operator): {writer.partial_path}')
            self._sink.detach()
        with self._lock:
            self._set_state('IDLE')

    def finish(self):
        """After motor cleanup, join camera and manager threads; no success prompt."""
        if self._finished:
            return
        self.request_stop()
        errors = []
        if self.runtime is not None:
            try:
                self.runtime.close()
            except Exception as error:
                errors.append(str(error))
        if self._thread is not None:
            retry_cleanup = not self._thread.is_alive() and self._manager_error is not None
            self._thread.join(300)
            if self._thread.is_alive():
                errors.append('background dataset finalization did not finish within 300 s')
            elif retry_cleanup:
                self._close_writers()
        elif self._prepared is not None:
            self._prepared.discard()
            self._prepared = None
        if self._manager_error:
            errors.append(self._manager_error)
        if errors:
            raise RuntimeError('; '.join(errors))
        self._finished = True
