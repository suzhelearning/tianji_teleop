"""Independent R/S/D episode management; never changes the robot's TELEOP state."""
from pathlib import Path
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
    def __init__(self, dataset_dir, task, config_path, model_path):
        try:
            from .dataset import EpisodeWriter
            from .runtime import validate_camera_profiles
        except ImportError as error:
            raise RuntimeError('collection dependencies unavailable; use collection.sh') from error
        if not isinstance(task, str) or not task.strip():
            raise ValueError('collection requires a nonempty task label')
        self.config, self.cameras = load_collection_config(config_path)
        validate_camera_profiles(self.cameras)
        self.metadata = dataset_metadata(self.config, self.cameras, model_path)
        self.dataset_dir = Path(dataset_dir).resolve()
        self.task = task
        self._writer_factory = EpisodeWriter
        # Prove the directory/config/HDF5 are writable before connecting motors.
        # This unused prepared file is discarded if R is never pressed.
        self._prepared = EpisodeWriter(self.dataset_dir, self.metadata, self.task)
        self._sink = _WriterSlot()
        self._lock = threading.Lock()
        self._commands = queue.Queue(maxsize=8)
        self._state = 'IDLE'
        self._closing = False
        self._finished = False
        self._last_error = None
        self._manager_error = None
        self._notice = None
        self._thread = None
        self.runtime = None
        self.saved_paths = []
        print(f'DATASET READY: {self.dataset_dir} task={task} cameras={self.cameras}; '
              'TELEOP keys: r=start, s=save success, d=discard current; no actions recorded', flush=True)

    @property
    def state(self):
        with self._lock:
            return self._state

    def wrap_hardware(self, hardware):
        from .runtime import LockedDevice
        return {name: LockedDevice(device) for name, device in hardware.items()}

    def start(self, hardware):
        from .runtime import CollectionRuntime
        self.runtime = CollectionRuntime(hardware, self.cameras, self._sink,
                                         state_rate_hz=self.config['state_rate_hz'])
        self.runtime.start()
        self._thread = threading.Thread(target=self._manage, name='episode-manager', daemon=True)
        self._thread.start()

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
                self._state = 'STARTING'
                self._commands.put_nowait('start')
            elif key in ('s', 'd') and self._state == 'RECORDING':
                # Freeze this segment immediately; saving/deletion remains asynchronous.
                self.runtime.end_episode()
                self._state = 'SAVING' if key == 's' else 'DISCARDING'
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
            if self.state == 'RECORDING':
                self.end_episode()

    def end_episode(self):
        """An unsaved segment ends at HOME/stop/fault and is kept as partial, not success."""
        with self._lock:
            if self._state in ('STARTING', 'RECORDING'):
                if self.runtime is not None:
                    self.runtime.end_episode()
                self._state = 'ABORTING'
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
                        self._state = 'FAILED'
                    elif self._state != 'ABORTING':
                        self._state = 'IDLE'
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
            self._state = 'FAILED' if errors else 'CLOSED'

    def _start_recording(self):
        with self._lock:
            if self._state != 'STARTING' or self._closing:
                return
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
                self._state = 'RECORDING'
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
            self._state = 'IDLE'

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
