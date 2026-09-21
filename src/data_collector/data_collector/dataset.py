"""Schema-v1 episode storage: one background HDF5 writer plus a read-only validator.

The writer owns exactly one episode file. HDF5 writes and optional legacy JPEG
encoding happen on its private thread; control-loop callers only hand over
already-captured data through a bounded queue. Nothing here opens hardware, the
SDK, ROS or the network.

Data contract (schema-v1.md):

* timestamps are supplied as absolute host monotonic ns and stored relative to
  the episode start; each stream is non-decreasing and its first value is >= 0
* state snapshots are actual feedback: arms 14 and hands 40 (rad), 54 columns
  following the fixed joint order
* RGB frames are uint8, stored uncompressed or as legacy JPEG byte arrays
* only the configured cameras get a group; an episode is a `.partial.h5` file
  until close + full validation + atomic rename to `.h5`
"""

from __future__ import annotations

import enum
from datetime import datetime
from itertools import count
import json
import operator
import os
import queue
import time
import threading
from pathlib import Path
from typing import Any, Final, Mapping, Sequence

import cv2
import h5py
import numpy as np

SCHEMA_VERSION: Final = 1
JOINT_UNIT: Final = 'rad'
IMAGE_ENCODINGS: Final = ('rgb', 'jpeg')
DECODED_COLOR_ORDER: Final = 'RGB'
STATE_DIM: Final = 54
ARM_DIM: Final = 14
HAND_DIM: Final = 40

DATASET_CONFIG_NAME: Final = 'dataset_config.json'
_TAKE_NUMBERS = count(1)

CONFIG_KEYS: Final = (
    'schema_version',
    'robot_config',
    'joint_names',
    'joint_unit',
    'policy_rate_hz',
    'camera_names',
    'image_width',
    'image_height',
    'image_encoding',
    'decoded_color_order',
    'jpeg_quality',
)

STATE_STREAMS: Final = (('arms', ARM_DIM), ('hands', HAND_DIM))

_STATE_CHUNK_ROWS: Final = 256
_IMAGE_CHUNK_ROWS: Final = 32
_STATE_BATCH_ROWS: Final = 256
_IMAGE_BATCH_ROWS: Final = 32
_OPEN_TIMEOUT_S: Final = 120.0
_CLOSE_TIMEOUT_S: Final = 120.0
_RESERVE_ATTEMPTS: Final = 4096


class EpisodeValidationError(RuntimeError):
    """An episode file does not satisfy the schema-v1 data contract."""


# --------------------------------------------------------------------------- #
# dataset-level config
# --------------------------------------------------------------------------- #


def _is_int(value: Any) -> bool:
    return isinstance(value, (int, np.integer)) and not isinstance(value, bool)


def normalize_dataset_config(config: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a dataset-level config mapping and return it as plain Python types.

    Raises ValueError for missing/unknown keys or values that violate schema-v1
    section 6. The returned dict has exactly the schema keys, in schema order.
    """
    if not isinstance(config, Mapping):
        raise ValueError(f'dataset_config must be a mapping, got {type(config).__name__}')
    missing = [key for key in CONFIG_KEYS if key not in config]
    unknown = sorted(set(config) - set(CONFIG_KEYS))
    if missing or unknown:
        raise ValueError(
            f'dataset_config keys must be exactly {list(CONFIG_KEYS)}; '
            f'missing={missing} unknown={unknown}'
        )

    if not (_is_int(config['schema_version']) and int(config['schema_version']) == SCHEMA_VERSION):
        raise ValueError(f'schema_version must be {SCHEMA_VERSION}, got {config["schema_version"]!r}')
    robot_config = config['robot_config']
    if not isinstance(robot_config, str) or not robot_config.strip():
        raise ValueError('robot_config must be a non-empty string')
    joint_names = config['joint_names']
    if not isinstance(joint_names, (list, tuple)) or len(joint_names) != STATE_DIM:
        raise ValueError(f'joint_names must list exactly {STATE_DIM} names')
    if not all(isinstance(name, str) and name.strip() for name in joint_names):
        raise ValueError('joint_names must be non-empty strings')
    if len(set(joint_names)) != STATE_DIM:
        raise ValueError('joint_names must be unique')
    if config['joint_unit'] != JOINT_UNIT:
        raise ValueError(f'joint_unit must be {JOINT_UNIT!r}, got {config["joint_unit"]!r}')
    rate = config['policy_rate_hz']
    if not isinstance(rate, (int, float)) or isinstance(rate, bool) or not np.isfinite(rate) or rate <= 0:
        raise ValueError(f'policy_rate_hz must be a positive number, got {rate!r}')
    if float(rate) != 30.0:
        raise ValueError(f'schema_version {SCHEMA_VERSION} requires policy_rate_hz 30, got {rate!r}')
    camera_names = config['camera_names']
    if not isinstance(camera_names, (list, tuple)) or not camera_names:
        raise ValueError('camera_names must be a non-empty ordered list')
    if not all(isinstance(name, str) and name.strip() for name in camera_names):
        raise ValueError('camera_names must be non-empty strings')
    if len(set(camera_names)) != len(camera_names):
        raise ValueError('camera_names must be unique')
    width, height = config['image_width'], config['image_height']
    if not (_is_int(width) and int(width) > 0) or not (_is_int(height) and int(height) > 0):
        raise ValueError(f'image_width/image_height must be positive integers, got {width!r}/{height!r}')
    encoding = config['image_encoding']
    if encoding not in IMAGE_ENCODINGS:
        raise ValueError(f'image_encoding must be one of {IMAGE_ENCODINGS!r}, got {encoding!r}')
    if config['decoded_color_order'] != DECODED_COLOR_ORDER:
        raise ValueError(
            f'decoded_color_order must be {DECODED_COLOR_ORDER!r}, got {config["decoded_color_order"]!r}'
        )
    quality = config['jpeg_quality']
    if encoding == 'rgb':
        if quality is not None:
            raise ValueError(f'jpeg_quality must be null for rgb images, got {quality!r}')
    elif not (_is_int(quality) and 1 <= int(quality) <= 100):
        raise ValueError(f'jpeg_quality must be an integer in [1, 100], got {quality!r}')

    return {
        'schema_version': SCHEMA_VERSION,
        'robot_config': str(robot_config),
        'joint_names': [str(name) for name in joint_names],
        'joint_unit': JOINT_UNIT,
        'policy_rate_hz': float(rate),
        'camera_names': [str(name) for name in camera_names],
        'image_width': int(width),
        'image_height': int(height),
        'image_encoding': encoding,
        'decoded_color_order': DECODED_COLOR_ORDER,
        'jpeg_quality': int(quality) if encoding == 'jpeg' else None,
    }


def load_dataset_config(dataset_dir: Path | str) -> dict[str, Any]:
    """Read and validate `<dataset_dir>/dataset_config.json`."""
    path = Path(dataset_dir) / DATASET_CONFIG_NAME
    try:
        raw = json.loads(path.read_text('utf-8'))
    except OSError as exc:
        raise ValueError(f'cannot read {path}: {exc}') from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f'{path} is not valid JSON: {exc}') from exc
    return normalize_dataset_config(raw)


def dataset_config_bytes(config: Mapping[str, Any]) -> bytes:
    """Canonical on-disk encoding of a dataset config."""
    return json.dumps(config, indent=2, sort_keys=True, ensure_ascii=False).encode('utf-8') + b'\n'


def ensure_dataset_config(dataset_dir: Path | str, config: Mapping[str, Any]) -> None:
    """Publish immutable root metadata atomically, or require an identical config."""
    normalized = normalize_dataset_config(config)
    directory = Path(dataset_dir)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / DATASET_CONFIG_NAME
    if not target.exists():
        temp = directory / f'.{DATASET_CONFIG_NAME}.{os.getpid()}.{threading.get_ident()}.tmp'
        try:
            with open(temp, 'wb') as handle:
                handle.write(dataset_config_bytes(normalized))
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.link(temp, target)
            except FileExistsError:
                # Another publisher won; compare its contract below.
                pass
            else:
                return
        finally:
            temp.unlink(missing_ok=True)
    try:
        existing = load_dataset_config(directory)
    except ValueError as exc:
        raise RuntimeError(f'existing {target} is not a valid dataset config: {exc}') from exc
    if existing != normalized:
        raise RuntimeError(
            f'existing {target} does not match the requested dataset config: '
            f'{existing} != {normalized}'
        )


# --------------------------------------------------------------------------- #
# writer
# --------------------------------------------------------------------------- #


class _State(enum.Enum):
    NEW = 'new'
    STARTED = 'started'
    STOPPED = 'stopped'
    CLOSING = 'closing'
    CLOSED = 'closed'


class EpisodeWriter:
    """Writes one schema-v1 episode through a single background HDF5 thread.

    The constructor reserves a dated timestamp/take `.partial.h5`, publishes the
    immutable dataset config in that take's date directory and waits for the
    worker to be ready, so initialization errors surface before hardware is enabled.

    If construction times out or is interrupted, cancellation is handed to the
    worker without waiting for blocked filesystem calls. Once opening returns,
    the worker closes and removes its never-started partial file.

    `append_*` methods never block, never touch disk and never raise: while the
    episode is recording they enqueue a sample and return True, or latch a
    fatal capture error (queue overflow, wrong joint count, non-finite values,
    unknown camera, wrong RGB geometry) that `check()` and `finish()` report.
    Samples from before the episode origin are discarded and counted. A
    backwards timestamp within an episode is a fatal capture discontinuity.
    Outside start()..stop(), appends return False without validation or counting, so a stopped writer can
    never be turned into a failed one by a late sampler thread.

    `finish()` closes the file, validates it in full and atomically renames it to
    `.h5`; `abort()` closes it and keeps the `.partial.h5` for forensics.
    """

    def __init__(
        self,
        dataset_dir: Path | str,
        dataset_config: Mapping[str, Any],
        task: str,
        queue_capacity: int = 64,
    ) -> None:
        self._config = normalize_dataset_config(dataset_config)
        if not isinstance(task, str) or not task.strip():
            raise ValueError('task must be a non-empty string')
        if not _is_int(queue_capacity) or int(queue_capacity) < 1:
            raise ValueError(f'queue_capacity must be a positive integer, got {queue_capacity!r}')

        self._dataset_dir = Path(dataset_dir)
        self._task = task
        self._queue: queue.Queue = queue.Queue(maxsize=int(queue_capacity))
        self._queue_capacity = int(queue_capacity)
        self._lock = threading.Lock()
        self._state = _State.NEW
        self._failure: BaseException | None = None
        self._rejected: dict[str, int] = {}
        self._start_ns: int | None = None
        self._last_relative: dict[str, int] = {}
        self._episode_number: int | None = None
        self._partial_path: Path | None = None
        self._ready = threading.Event()
        self._closed = threading.Event()
        self._initialization_done = threading.Event()
        self._initialization_cancelled = False
        self._close_requested = False
        self._close_success: bool | None = None
        self._file: h5py.File | None = None
        self._streams: dict[str, tuple[h5py.Dataset, h5py.Dataset]] = {}
        self._pending_ts: dict[str, list[int]] = {}
        self._pending_values: dict[str, list[Any]] = {}

        self._thread = threading.Thread(target=self._run, name='episode-writer', daemon=True)
        try:
            self._thread.start()
            if not self._ready.wait(timeout=_OPEN_TIMEOUT_S):
                raise RuntimeError(f'episode writer did not become ready within {_OPEN_TIMEOUT_S:.0f}s')
            self.check()
        except BaseException:
            self._initialization_cancelled = True
            raise
        finally:
            self._initialization_done.set()

    # -- public control ---------------------------------------------------- #

    @property
    def partial_path(self) -> Path:
        """Path of the `.partial.h5` file reserved by this writer."""
        return self._partial_path

    @property
    def final_path(self) -> Path:
        """Target path of a successful `finish()`."""
        return self._partial_path.with_name(self._partial_path.name.replace('.partial.h5', '.h5'))

    @property
    def episode_number(self) -> int:
        return self._episode_number

    @property
    def rejected_counts(self) -> dict[str, int]:
        """Per-stream count of dropped samples (negative or backwards timestamps)."""
        with self._lock:
            return dict(self._rejected)

    def start(self, start_ns: int) -> None:
        """Begin capture. `start_ns` is the host monotonic episode origin."""
        if not _is_int(start_ns):
            raise TypeError('start_ns must be an integer host timestamp')
        origin = int(start_ns)
        if origin <= 0:
            raise ValueError('start_ns must be positive')
        with self._lock:
            if self._state is not _State.NEW:
                raise RuntimeError(f'start() is only valid once, writer is {self._state.value}')
            self._start_ns = origin
            self._state = _State.STARTED

    def stop(self) -> None:
        """Stop accepting samples. Non-blocking; queued samples are still written."""
        with self._lock:
            if self._state in (_State.NEW, _State.STARTED):
                self._state = _State.STOPPED

    def check(self) -> None:
        """Raise RuntimeError if the writer thread or the capture queue failed."""
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise RuntimeError(f'episode writer failed: {failure}') from failure

    def finish(self, success: bool) -> Path:
        """Close, validate and atomically rename the take to `.h5`.

        `success` is the operator's task verdict and is stored as the episode
        attribute; a failed task still produces a complete, valid episode.
        """
        if not isinstance(success, bool):
            raise TypeError(f'success must be bool, got {type(success).__name__}')
        self.check()
        with self._lock:
            if self._state is _State.CLOSED:
                raise RuntimeError(f'episode {self._partial_path} is already closed')
            if self._state is not _State.STOPPED:
                raise RuntimeError(f'finish() requires stop() first, writer is {self._state.value}')
            self._state = _State.CLOSING
        self._request_close(success)
        self.check()
        validate_episode(self._partial_path, self._config)
        target = self.final_path
        if target.exists():
            raise RuntimeError(f'refusing to overwrite existing episode {target}')
        # Link publication is atomic and cannot overwrite an episode that
        # another process published after the existence check.
        os.link(self._partial_path, target)
        self._partial_path.unlink()
        return target

    def abort(self) -> Path | None:
        """Close and retain the partial, even after capture errors.

        Raise on close timeout; a later call may retry once the worker unblocks.
        """
        with self._lock:
            if self._state is not _State.CLOSED:
                self._state = _State.CLOSING
        self._request_close(None)
        return self._partial_path

    def discard(self) -> None:
        """Close and delete only this writer's unfinished episode, never a saved file."""
        self.stop()
        self.abort()
        self._thread.join(_CLOSE_TIMEOUT_S)
        if self._thread.is_alive():
            raise RuntimeError('cannot discard an episode while its writer is still running')
        if self._partial_path is not None:
            self._partial_path.unlink(missing_ok=True)

    # -- public producers -------------------------------------------------- #

    def append_arms(self, timestamp_ns: int, qpos: Sequence[float] | np.ndarray) -> bool:
        """Queue one 14-dim arm feedback snapshot. Returns False if not accepted."""
        return self._append_state('arms', timestamp_ns, qpos, ARM_DIM)

    def append_hands(self, timestamp_ns: int, qpos: Sequence[float] | np.ndarray) -> bool:
        """Queue one 40-dim hand feedback snapshot. Returns False if not accepted."""
        return self._append_state('hands', timestamp_ns, qpos, HAND_DIM)

    def append_rgb(self, name: str, timestamp_ns: int, rgb: np.ndarray) -> bool:
        """Queue one RGB frame for `name` (uint8, configured HxWx3, true RGB order).

        The frame is not copied: the caller keeps ownership until the writer has
        consumed it and MUST NOT mutate the buffer while it is queued.
        """
        height, width = self._config['image_height'], self._config['image_width']
        with self._lock:
            if self._state is not _State.STARTED:
                return False
            if name not in self._config['camera_names']:
                self._latch_failure_locked(
                    ValueError(
                        f'unknown camera {name!r}; configured cameras are {self._config["camera_names"]}'
                    )
                )
                return False
            timestamp = self._index_timestamp_locked(name, timestamp_ns)
            if timestamp is None:
                return False
            try:
                frame = np.asarray(rgb)
            except Exception as exc:  # noqa: BLE001 - latched, surfaces via check()
                self._latch_failure_locked(ValueError(f'{name}: RGB frame is not array-like: {exc}'))
                return False
            if frame.dtype != np.uint8 or frame.shape != (height, width, 3):
                self._latch_failure_locked(
                    ValueError(
                        f'{name}: RGB frame must be uint8 ({height}, {width}, 3), '
                        f'got {frame.dtype} {frame.shape}'
                    )
                )
                return False
            return self._offer_locked(('rgb', name, timestamp, frame))

    # -- append plumbing --------------------------------------------------- #

    def _append_state(self, stream: str, timestamp_ns: int, qpos: Any, dim: int) -> bool:
        with self._lock:
            if self._state is not _State.STARTED:
                return False
            timestamp = self._index_timestamp_locked(stream, timestamp_ns)
            if timestamp is None:
                return False
            try:
                values = np.array(qpos, dtype=np.float32)
            except (TypeError, ValueError) as exc:
                self._latch_failure_locked(ValueError(f'{stream}: qpos is not a numeric sequence: {exc}'))
                return False
            if values.shape != (dim,):
                self._latch_failure_locked(ValueError(f'{stream}: qpos must have shape ({dim},), got {values.shape}'))
                return False
            if not np.isfinite(values).all():
                self._latch_failure_locked(ValueError(f'{stream}: qpos contains non-finite values'))
                return False
            return self._offer_locked((stream, timestamp, values))

    def _index_timestamp_locked(self, stream: str, timestamp_ns: Any) -> int | None:
        try:
            return operator.index(timestamp_ns)
        except TypeError:
            self._latch_failure_locked(
                TypeError(f'{stream}: timestamp_ns must be an integer, got {type(timestamp_ns).__name__}')
            )
            return None

    def _offer_locked(self, item: tuple) -> bool:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._latch_failure_locked(
                RuntimeError(
                    f'capture queue overflow (capacity {self._queue_capacity}): '
                    'the writer cannot keep up without dropping data'
                )
            )
            return False
        return True

    def _latch_failure(self, error: BaseException) -> None:
        with self._lock:
            self._latch_failure_locked(error)

    def _latch_failure_locked(self, error: BaseException) -> None:
        if self._failure is None:
            self._failure = error

    # -- worker ------------------------------------------------------------ #

    def _run(self) -> None:
        try:
            self._open_episode()
        except BaseException as error:  # noqa: BLE001 - surfaced through check()
            self._latch_failure(error)
        self._ready.set()
        # The constructor acknowledges readiness or cancellation before the
        # worker can start waiting for samples. This covers interruption on
        # either side of _ready.set(), including a delayed filesystem open.
        self._initialization_done.wait()
        if self._initialization_cancelled:
            try:
                if self._file is not None:
                    self._file.close()
                    self._file = None
                # Only _reserve_episode()'s successful exclusive reservation
                # publishes this path; existing takes are never cleanup targets.
                if self._partial_path is not None:
                    self._partial_path.unlink(missing_ok=True)
            except BaseException as error:  # noqa: BLE001
                self._latch_failure(error)
            finally:
                with self._lock:
                    self._state = _State.CLOSED
                self._closed.set()
            return

        success: bool | None = None
        while True:
            with self._lock:
                if self._close_requested and self._queue.empty():
                    success = self._close_success
                    break
            item = self._queue.get()
            try:
                if item[0] == 'close':
                    success = item[1]
                    break
                self._handle(item)
            except BaseException as error:  # noqa: BLE001 - latched, surfaces via check()
                self._latch_failure(error)
                break
            finally:
                self._queue.task_done()

        try:
            self._close_episode(success)
        except BaseException as error:  # noqa: BLE001
            self._latch_failure(error)
        finally:
            with self._lock:
                self._state = _State.CLOSED
            self._closed.set()

    def _open_episode(self) -> None:
        number, path = self._reserve_episode()
        try:
            ensure_dataset_config(path.parent, self._config)
        except BaseException:
            path.unlink(missing_ok=True)
            raise
        self._episode_number = number
        self._partial_path = path

        episode = h5py.File(path, 'w')
        try:
            episode.attrs['schema_version'] = int(self._config['schema_version'])
            episode.attrs['task'] = self._task
            episode.attrs['robot_config'] = self._config['robot_config']
            observations = episode.create_group('observations')
            for stream, dim in STATE_STREAMS:
                parent = observations.create_group(stream)
                self._streams[stream] = (
                    parent.create_dataset(
                        'timestamp_ns',
                        shape=(0,),
                        maxshape=(None,),
                        dtype=np.int64,
                        chunks=(_STATE_CHUNK_ROWS,),
                    ),
                    parent.create_dataset(
                        'qpos',
                        shape=(0, dim),
                        maxshape=(None, dim),
                        dtype=np.float32,
                        chunks=(_STATE_CHUNK_ROWS, dim),
                    ),
                )
                self._pending_ts[stream] = []
                self._pending_values[stream] = []
            images = episode.create_group('images')
            for camera in self._config['camera_names']:
                group = images.create_group(camera)
                if self._config['image_encoding'] == 'rgb':
                    frame_shape = (self._config['image_height'], self._config['image_width'], 3)
                    frames = group.create_dataset(
                        'rgb',
                        shape=(0, *frame_shape),
                        maxshape=(None, *frame_shape),
                        dtype=np.uint8,
                        chunks=(1, *frame_shape),
                    )
                else:
                    frames = group.create_dataset(
                        'jpeg',
                        shape=(0,),
                        maxshape=(None,),
                        dtype=h5py.vlen_dtype(np.uint8),
                        chunks=(_IMAGE_CHUNK_ROWS,),
                    )
                self._streams[camera] = (
                    group.create_dataset(
                        'timestamp_ns',
                        shape=(0,),
                        maxshape=(None,),
                        dtype=np.int64,
                        chunks=(_IMAGE_CHUNK_ROWS,),
                    ),
                    frames,
                )
                self._pending_ts[camera] = []
                self._pending_values[camera] = []
        except BaseException:
            episode.close()
            raise
        self._file = episode
        episode.flush()

    def _reserve_episode(self) -> tuple[int, Path]:
        stamp = datetime.fromtimestamp(time.time_ns() / 1e9).strftime('%Y%m%d_%H%M%S_%f')
        directory = self._dataset_dir / stamp[:8]
        directory.mkdir(parents=True, exist_ok=True)
        for _ in range(_RESERVE_ATTEMPTS):
            number = next(_TAKE_NUMBERS)
            stem = f'{stamp}_take{number:03d}'
            path = directory / f'{stem}.partial.h5'
            try:
                handle = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            except FileExistsError:
                continue
            os.close(handle)
            if (directory / f'{stem}.h5').exists():
                path.unlink()
                continue
            return number, path
        raise RuntimeError(f'cannot reserve a unique take in {directory}')

    def _handle(self, item: tuple) -> None:
        if item[0] == 'rgb':
            _, camera, timestamp_ns, frame = item
            relative = self._relative_timestamp(camera, timestamp_ns)
            if relative is None:
                return
            if self._config['image_encoding'] == 'rgb':
                # Write one frame directly: no encoding, batch stacking or extra
                # frame retention beyond the bounded producer queue.
                timestamp_ds, rgb_ds = self._streams[camera]
                index = timestamp_ds.shape[0]
                timestamp_ds.resize((index + 1,))
                rgb_ds.resize((index + 1, *rgb_ds.shape[1:]))
                timestamp_ds[index] = relative
                rgb_ds[index] = frame
                return
            self._pending_ts[camera].append(relative)
            self._pending_values[camera].append(self._encode_jpeg(camera, frame))
            if len(self._pending_ts[camera]) >= _IMAGE_BATCH_ROWS:
                self._flush_images(camera)
            return
        stream, timestamp_ns, values = item
        relative = self._relative_timestamp(stream, timestamp_ns)
        if relative is None:
            return
        self._pending_ts[stream].append(relative)
        self._pending_values[stream].append(values)
        if len(self._pending_ts[stream]) >= _STATE_BATCH_ROWS:
            self._flush_state(stream)

    def _relative_timestamp(self, stream: str, timestamp_ns: int) -> int | None:
        relative = timestamp_ns - self._start_ns
        previous = self._last_relative.get(stream)
        if relative < 0:
            with self._lock:
                self._rejected[stream] = self._rejected.get(stream, 0) + 1
            return None
        if previous is not None and relative < previous:
            raise ValueError(f'{stream}: timestamp regressed within an episode ({relative} < {previous})')
        self._last_relative[stream] = relative
        return relative

    def _encode_jpeg(self, camera: str, frame: np.ndarray) -> np.ndarray:
        # Callers hand over true RGB (SDK RGB8). OpenCV encodes channel 0 as JPEG
        # blue, so convert: the stored file then decodes to the documented RGB order.
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(
            '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self._config['jpeg_quality'])]
        )
        if not ok:
            raise RuntimeError(f'{camera}: JPEG encoding failed')
        return np.ascontiguousarray(encoded).reshape(-1)

    def _request_close(self, success: bool | None) -> None:
        with self._lock:
            if not self._close_requested and not self._closed.is_set():
                self._close_requested = True
                self._close_success = success
                try:
                    self._queue.put_nowait(('close', success))
                except queue.Full:
                    # A full queue needs no wakeup: the worker observes the
                    # request after draining accepted samples.
                    pass
        if not self._closed.wait(timeout=_CLOSE_TIMEOUT_S):
            error = RuntimeError(f'episode writer did not close within {_CLOSE_TIMEOUT_S:.0f}s')
            self._latch_failure(error)
            raise error

    def _close_episode(self, success: bool | None) -> None:
        episode = self._file
        if episode is None:
            return
        error: BaseException | None = None
        try:
            self._flush_all()
            if success is not None:
                episode.attrs['success'] = bool(success)
            episode.flush()
        except BaseException as exc:  # noqa: BLE001 - re-raised after close
            error = exc
        finally:
            try:
                episode.close()
            except BaseException as exc:  # noqa: BLE001
                error = error or exc
            self._file = None
        if error is not None:
            raise error

    def _flush_state(self, stream: str) -> None:
        timestamps = self._pending_ts[stream]
        if not timestamps:
            return
        values = self._pending_values[stream]
        timestamp_ds, qpos_ds = self._streams[stream]
        start = int(timestamp_ds.shape[0])
        count = len(timestamps)
        timestamp_ds.resize((start + count,))
        qpos_ds.resize((start + count, qpos_ds.shape[1]))
        timestamp_ds[start:start + count] = np.asarray(timestamps, dtype=np.int64)
        qpos_ds[start:start + count] = np.stack(values)
        del timestamps[:]
        del values[:]

    def _flush_images(self, camera: str) -> None:
        timestamps = self._pending_ts[camera]
        if not timestamps:
            return
        frames = self._pending_values[camera]
        timestamp_ds, jpeg_ds = self._streams[camera]
        start = int(timestamp_ds.shape[0])
        count = len(timestamps)
        timestamp_ds.resize((start + count,))
        jpeg_ds.resize((start + count,))
        timestamp_ds[start:start + count] = np.asarray(timestamps, dtype=np.int64)
        jpeg_ds[start:start + count] = frames
        del timestamps[:]
        del frames[:]

    def _flush_all(self) -> None:
        for stream, _ in STATE_STREAMS:
            self._flush_state(stream)
        for camera in self._config['camera_names']:
            self._flush_images(camera)


# --------------------------------------------------------------------------- #
# read-only validation
# --------------------------------------------------------------------------- #


def _attribute_text(attrs: h5py.AttributeManager, name: str, label: str) -> str:
    if name not in attrs:
        raise EpisodeValidationError(f'{label}: missing attribute {name!r}')
    value = attrs[name]
    if isinstance(value, bytes):
        value = value.decode('utf-8', 'strict')
    if not isinstance(value, str) or not value:
        raise EpisodeValidationError(f'{label}: attribute {name!r} must be a non-empty string')
    return value


def _dataset(group: h5py.Group, name: str, label: str) -> h5py.Dataset:
    if name not in group:
        raise EpisodeValidationError(f'{label}: missing dataset {name!r}')
    value = group[name]
    if not isinstance(value, h5py.Dataset):
        raise EpisodeValidationError(f'{label}: {name!r} is not a dataset')
    return value


def _group(parent: h5py.Group, name: str, label: str) -> h5py.Group:
    if name not in parent:
        raise EpisodeValidationError(f'{label}: missing group {name!r}')
    value = parent[name]
    if not isinstance(value, h5py.Group):
        raise EpisodeValidationError(f'{label}: {name!r} is not a group')
    return value


def _check_members(group: h5py.Group, expected: set[str], label: str) -> None:
    actual = set(group.keys())
    if actual != expected:
        raise EpisodeValidationError(
            f'{label}: expected exactly {sorted(expected)}, found {sorted(actual)}'
        )


def _read_timestamps(dataset: h5py.Dataset, label: str) -> np.ndarray:
    if dataset.dtype != np.int64 or dataset.ndim != 1:
        raise EpisodeValidationError(f'{label}: timestamp_ns must be int64 [N], got {dataset.dtype} {dataset.shape}')
    if dataset.shape[0] == 0:
        raise EpisodeValidationError(f'{label}: timestamp_ns is empty')
    timestamps = dataset[()]
    if int(timestamps[0]) < 0:
        raise EpisodeValidationError(f'{label}: first timestamp {int(timestamps[0])} is negative')
    if timestamps.size > 1 and bool((np.diff(timestamps) < 0).any()):
        raise EpisodeValidationError(f'{label}: timestamps are not non-decreasing')
    return timestamps


def _validate_state_stream(
    episode: h5py.File, group_path: str, dim: int, label: str
) -> np.ndarray:
    group = _group(episode, group_path, label)
    _check_members(group, {'timestamp_ns', 'qpos'}, label)
    timestamps = _read_timestamps(_dataset(group, 'timestamp_ns', label), label)
    qpos_ds = _dataset(group, 'qpos', label)
    if qpos_ds.dtype != np.float32 or qpos_ds.ndim != 2 or qpos_ds.shape[1] != dim:
        raise EpisodeValidationError(
            f'{label}: qpos must be float32 [N,{dim}], got {qpos_ds.dtype} {qpos_ds.shape}'
        )
    if qpos_ds.shape[0] != timestamps.size:
        raise EpisodeValidationError(
            f'{label}: {qpos_ds.shape[0]} qpos rows do not match {timestamps.size} timestamps'
        )
    values = qpos_ds[()]
    if not np.isfinite(values).all():
        raise EpisodeValidationError(f'{label}: qpos contains non-finite values')
    return timestamps


def _validate_image_stream(
    episode: h5py.File, camera: str, config: Mapping[str, Any], decode: bool
) -> np.ndarray:
    label = f'images/{camera}'
    group = _group(episode, label, label)
    encoding = config['image_encoding']
    _check_members(group, {'timestamp_ns', encoding}, label)
    timestamps = _read_timestamps(_dataset(group, 'timestamp_ns', label), label)
    height, width = int(config['image_height']), int(config['image_width'])
    if encoding == 'rgb':
        rgb_ds = _dataset(group, 'rgb', label)
        expected = (timestamps.size, height, width, 3)
        if rgb_ds.dtype != np.uint8 or rgb_ds.shape != expected:
            raise EpisodeValidationError(
                f'{label}: rgb must be uint8 {expected}, got {rgb_ds.dtype} {rgb_ds.shape}'
            )
        if rgb_ds.chunks != (1, height, width, 3):
            raise EpisodeValidationError(f'{label}: rgb must use one-frame chunks (1, {height}, {width}, 3)')
        if rgb_ds.id.get_create_plist().get_nfilters() != 0:
            raise EpisodeValidationError(f'{label}: rgb must be uncompressed with no HDF5 filters')
        return timestamps
    jpeg_ds = _dataset(group, 'jpeg', label)
    if jpeg_ds.ndim != 1 or h5py.check_vlen_dtype(jpeg_ds.dtype) != np.uint8:
        raise EpisodeValidationError(f'{label}: jpeg must be vlen uint8 [N], got {jpeg_ds.dtype} {jpeg_ds.shape}')
    if jpeg_ds.shape[0] != timestamps.size:
        raise EpisodeValidationError(
            f'{label}: {jpeg_ds.shape[0]} frames do not match {timestamps.size} timestamps'
        )
    for index in range(timestamps.size):
        raw = jpeg_ds[index]
        if raw.size < 2 or raw[0] != 0xFF or raw[1] != 0xD8:
            raise EpisodeValidationError(f'{label}: frame {index} is not a JPEG byte stream')
        if not decode:
            continue
        image = cv2.imdecode(np.asarray(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise EpisodeValidationError(f'{label}: frame {index} is not decodable')
        if image.shape != (height, width, 3):
            raise EpisodeValidationError(
                f'{label}: frame {index} decoded to {image.shape}, expected ({height}, {width}, 3)'
            )
    return timestamps


def _rate_hz(timestamps: np.ndarray) -> float | None:
    if timestamps.size < 2:
        return None
    span = int(timestamps[-1]) - int(timestamps[0])
    if span <= 0:
        return None
    return (timestamps.size - 1) * 1e9 / span


def validate_episode(
    path: Path | str, dataset_config: Mapping[str, Any], *, require_success: bool = True
) -> dict[str, Any]:
    """Fully validate one schema-v1 episode file; read-only.

    Raises EpisodeValidationError when attributes, groups, dtypes, dimensions,
    value finiteness, timestamp order or JPEG decodability violate the schema.
    Returns counts, first/last timestamps, per-stream rates measured from the
    stored timestamps and the episode duration. Nothing is added to the file.
    """
    config = normalize_dataset_config(dataset_config)
    episode_path = Path(path)
    if not episode_path.is_file():
        raise EpisodeValidationError(f'episode file not found: {episode_path}')

    streams: dict[str, np.ndarray] = {}
    with h5py.File(episode_path, 'r') as episode:
        label = str(episode_path)
        _check_members(episode, {'observations', 'images'}, label)
        if 'schema_version' not in episode.attrs:
            raise EpisodeValidationError(f'{label}: missing attribute schema_version')
        if int(episode.attrs['schema_version']) != SCHEMA_VERSION:
            raise EpisodeValidationError(
                f'{label}: schema_version {int(episode.attrs["schema_version"])} != {SCHEMA_VERSION}'
            )
        robot_config = _attribute_text(episode.attrs, 'robot_config', label)
        if robot_config != config['robot_config']:
            raise EpisodeValidationError(
                f'{label}: robot_config {robot_config!r} does not match dataset config {config["robot_config"]!r}'
            )
        task = _attribute_text(episode.attrs, 'task', label)
        success: bool | None = None
        if 'success' in episode.attrs:
            success = bool(episode.attrs['success'])
        elif require_success:
            raise EpisodeValidationError(f'{label}: missing attribute success')

        observations = _group(episode, 'observations', label)
        _check_members(observations, {'arms', 'hands'}, label)
        streams['arms'] = _validate_state_stream(episode, 'observations/arms', ARM_DIM, 'observations/arms')
        streams['hands'] = _validate_state_stream(episode, 'observations/hands', HAND_DIM, 'observations/hands')

        images = _group(episode, 'images', label)
        _check_members(images, set(config['camera_names']), 'images')
        for camera in config['camera_names']:
            streams[camera] = _validate_image_stream(episode, camera, config, True)

    counts = {name: int(times.size) for name, times in streams.items()}
    rates = {name: _rate_hz(times) for name, times in streams.items()}
    first = {name: int(times[0]) for name, times in streams.items()}
    last = {name: int(times[-1]) for name, times in streams.items()}
    return {
        'path': str(episode_path),
        'schema_version': SCHEMA_VERSION,
        'task': task,
        'success': success,
        'robot_config': config['robot_config'],
        'cameras': list(config['camera_names']),
        'counts': {
            'arms': counts['arms'],
            'hands': counts['hands'],
            'images': {camera: counts[camera] for camera in config['camera_names']},
        },
        'timestamps_ns': {name: {'first': first[name], 'last': last[name]} for name in streams},
        'rates_hz': rates,
        'duration_s': max(last.values()) / 1e9,
    }
