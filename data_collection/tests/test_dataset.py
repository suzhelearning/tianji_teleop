"""Offline tests for the schema-v1 episode writer and validator (no hardware)."""

from __future__ import annotations

import json
from datetime import datetime
from itertools import count
import sys
from pathlib import Path

import h5py
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from data_collection import dataset  # noqa: E402
from data_collection.dataset import (  # noqa: E402
    DATASET_CONFIG_NAME,
    EpisodeValidationError,
    EpisodeWriter,
    load_dataset_config,
    normalize_dataset_config,
    validate_episode,
)

TASK = 'pick_place'
START_NS = 5_000_000_000
HEIGHT, WIDTH = 720, 1280


def make_config(**overrides) -> dict:
    config = {
        'schema_version': 1,
        'robot_config': 'tianji_wuji2_v1',
        'joint_names': [f'joint_{index}' for index in range(54)],
        'joint_unit': 'rad',
        'policy_rate_hz': 30,
        'camera_names': ['top', 'left_wrist'],
        'image_width': WIDTH,
        'image_height': HEIGHT,
        'image_encoding': 'jpeg',
        'decoded_color_order': 'RGB',
        'jpeg_quality': 90,
    }
    config.update(overrides)
    return config


def frame(level: int) -> np.ndarray:
    """Solid RGB frame that encodes/decodes quickly and identifies its source row."""
    image = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    image[:, :, 0] = level % 200 + 20
    image[:, :, 1] = 100
    image[:, :, 2] = (level * 7) % 200 + 20
    return image


def fill(writer: EpisodeWriter, rows: int, *, arms_offset: int = 0) -> None:
    for index in range(rows):
        stamp = START_NS + index * 8_333_333
        assert writer.append_arms(stamp, np.full(14, index + arms_offset, np.float32))
        assert writer.append_hands(stamp, np.full(40, index, np.float32))
        assert writer.append_rgb('top', stamp, frame(index))
        assert writer.append_rgb('left_wrist', stamp, frame(index))


@pytest.fixture
def raw_episode(tmp_path, monkeypatch):
    def codec_unavailable(*args, **kwargs):
        raise RuntimeError('JPEG codec unavailable')

    monkeypatch.setattr(dataset.cv2, 'imencode', codec_unavailable)
    config = make_config(
        image_encoding='rgb', jpeg_quality=None,
        camera_names=['top'], image_height=4, image_width=6,
    )
    # Non-contiguous RGB input also must survive without channel swaps or loss.
    frames = np.arange(2 * 4 * 6 * 3, dtype=np.uint8).reshape(2, 4, 6, 3)[:, :, ::-1, :]
    writer = EpisodeWriter(tmp_path, config, TASK)
    try:
        writer.start(START_NS)
        for index, image in enumerate(frames):
            stamp = START_NS + index * 10
            assert writer.append_arms(stamp, np.full(14, index, np.float32))
            assert writer.append_hands(stamp, np.full(40, index, np.float32))
            assert writer.append_rgb('top', stamp, image)
        writer.stop()
        path = writer.finish(False)
    finally:
        writer.abort()
    return path, config, frames


def test_raw_exact_pixels_without_a_working_jpeg_codec(raw_episode):
    path, config, frames = raw_episode
    report = validate_episode(path, config)
    assert report['success'] is False
    assert report['counts'] == {'arms': 2, 'hands': 2, 'images': {'top': 2}}
    with h5py.File(path, 'r') as episode:
        camera = episode['images/top']
        assert set(camera) == {'timestamp_ns', 'rgb'}
        rgb = camera['rgb']
        assert rgb.dtype == np.uint8
        assert rgb.chunks == (1, 4, 6, 3)
        assert rgb.id.get_create_plist().get_nfilters() == 0
        np.testing.assert_array_equal(rgb[:], frames)
        np.testing.assert_array_equal(camera['timestamp_ns'][:], [0, 10])
        np.testing.assert_array_equal(episode['observations/arms/qpos'][:, 0], [0, 1])
        np.testing.assert_array_equal(episode['observations/hands/qpos'][:, 0], [0, 1])
    with pytest.raises(EpisodeValidationError, match='expected exactly'):
        validate_episode(path, {**config, 'image_encoding': 'jpeg', 'jpeg_quality': 50})


@pytest.mark.parametrize('layout', [
    {'shape': (2, 4, 6, 4), 'chunks': (1, 4, 6, 4)},
    {'dtype': np.float32},
    {'compression': 'gzip'},
    {'shuffle': True},
    {'fletcher32': True},
    {'chunks': (2, 4, 6, 3)},
    {'chunks': None},
], ids=['shape', 'dtype', 'compression', 'shuffle', 'checksum', 'multi-frame-chunks', 'contiguous'])
def test_validator_rejects_invalid_raw_storage(raw_episode, layout):
    path, config, _ = raw_episode
    with h5py.File(path, 'r+') as episode:
        camera = episode['images/top']
        del camera['rgb']
        options = {'shape': (2, 4, 6, 3), 'dtype': np.uint8, 'chunks': (1, 4, 6, 3)}
        options.update(layout)
        camera.create_dataset('rgb', **options)
    with pytest.raises(EpisodeValidationError, match='rgb'):
        validate_episode(path, config)


def test_partial_then_complete_roundtrip(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    assert writer.partial_path == writer.final_path.with_suffix('.partial.h5')
    assert writer.partial_path.is_file()
    assert writer.final_path.parent.parent == tmp_path
    written = json.loads((tmp_path / DATASET_CONFIG_NAME).read_text('utf-8'))
    assert normalize_dataset_config(written) == normalize_dataset_config(config)

    writer.start(START_NS)
    fill(writer, 4)
    writer.stop()
    final_path = writer.finish(True)

    assert final_path == writer.final_path
    assert final_path.is_file()
    assert not writer.partial_path.exists()

    with h5py.File(final_path, 'r') as episode:
        assert int(episode.attrs['schema_version']) == 1
        assert episode.attrs['task'] == TASK
        assert bool(episode.attrs['success']) is True
        assert episode.attrs['robot_config'] == 'tianji_wuji2_v1'
        assert 'action_type' not in episode.attrs
        assert set(episode.attrs.keys()) == {'schema_version', 'task', 'success', 'robot_config'}
        assert set(episode.keys()) == {'observations', 'images'}
        assert set(episode['observations'].keys()) == {'arms', 'hands'}
        assert set(episode['observations/arms'].keys()) == {'timestamp_ns', 'qpos'}
        assert set(episode['images'].keys()) == {'top', 'left_wrist'}

        for path, dim in (
            ('observations/arms', 14),
            ('observations/hands', 40),
        ):
            stamps = episode[f'{path}/timestamp_ns']
            values = episode[f'{path}/qpos']
            assert stamps.dtype == np.int64
            assert values.dtype == np.float32
            assert stamps.shape == (4,)
            assert values.shape == (4, dim)
            assert stamps.maxshape == (None,)
            assert list(stamps[()]) == [0, 8_333_333, 16_666_666, 24_999_999]
            np.testing.assert_allclose(values, np.tile(np.arange(4, dtype=np.float32)[:, None], (1, dim)))

        for camera in ('top', 'left_wrist'):
            stamps = episode[f'images/{camera}/timestamp_ns']
            jpeg = episode[f'images/{camera}/jpeg']
            assert stamps.dtype == np.int64
            assert jpeg.shape == (4,)
            assert h5py.check_vlen_dtype(jpeg.dtype) == np.uint8
            assert jpeg[1].dtype == np.uint8 and jpeg[1].ndim == 1

    report = validate_episode(final_path, config)
    assert report['success'] is True
    assert report['task'] == TASK
    assert report['counts'] == {
        'arms': 4,
        'hands': 4,
        'images': {'top': 4, 'left_wrist': 4},
    }
    assert 'action_type' not in report
    assert report['cameras'] == ['top', 'left_wrist']
    assert report['rates_hz']['arms'] == pytest.approx(120.0)
    assert report['rates_hz']['top'] == pytest.approx(120.0)
    assert report['duration_s'] == pytest.approx(0.025, abs=1e-6)


def test_operator_failed_but_complete_episode(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    writer.start(START_NS)
    fill(writer, 2)
    writer.stop()
    with pytest.raises(TypeError):
        writer.finish(1)  # not a bool: nothing is closed or renamed
    final_path = writer.finish(False)

    with h5py.File(final_path, 'r') as episode:
        assert bool(episode.attrs['success']) is False
    report = validate_episode(final_path, config)
    assert report['success'] is False
    assert report['counts']['arms'] == 2


def test_finish_requires_stop_and_refuses_empty_episode(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    with pytest.raises(RuntimeError, match='stop'):
        writer.finish(True)

    writer.start(START_NS)
    with pytest.raises(RuntimeError, match='stop'):
        writer.finish(True)
    writer.stop()
    with pytest.raises(EpisodeValidationError, match='empty'):
        writer.finish(True)
    assert writer.partial_path.is_file()
    assert not writer.final_path.exists()
    writer.abort()


@pytest.mark.parametrize('conflicting', [
    make_config(camera_names=['top']),
    make_config(image_encoding='rgb', jpeg_quality=None),
], ids=['camera-mismatch', 'encoding-mismatch'])
def test_config_conflict_is_rejected_without_touching_files(tmp_path: Path, conflicting) -> None:
    target = tmp_path / DATASET_CONFIG_NAME
    target.write_text(json.dumps(conflicting, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    original = target.read_bytes()

    with pytest.raises(RuntimeError, match='does not match'):
        EpisodeWriter(tmp_path, make_config(), TASK)
    assert target.read_bytes() == original
    assert not list(tmp_path.rglob('*.h5'))

    with pytest.raises(ValueError, match='unknown'):
        EpisodeWriter(tmp_path, make_config(extra='nope'), TASK)
    with pytest.raises(ValueError, match='joint_names'):
        EpisodeWriter(tmp_path, make_config(joint_names=['a', 'b']), TASK)
    with pytest.raises(ValueError, match='joint_unit'):
        EpisodeWriter(tmp_path, make_config(joint_unit='deg'), TASK)


def test_timestamp_takes_never_overwrite(tmp_path: Path, monkeypatch) -> None:
    now = datetime(2026, 9, 13, 12, 34, 56, 123456)
    monkeypatch.setattr(dataset.time, 'time_ns', lambda: int(now.timestamp() * 1e9))
    monkeypatch.setattr(dataset, '_TAKE_NUMBERS', count(1))
    directory = tmp_path / '20260913'
    directory.mkdir()
    completed = directory / '20260913_123456_123456_take001.h5'
    interrupted = directory / '20260913_123456_123456_take002.partial.h5'
    completed.write_bytes(b'completed')
    interrupted.write_bytes(b'interrupted')

    first = EpisodeWriter(tmp_path, make_config(), TASK)
    second = EpisodeWriter(tmp_path, make_config(), TASK)
    assert first.final_path == directory / '20260913_123456_123456_take003.h5'
    assert second.final_path == directory / '20260913_123456_123456_take004.h5'

    for writer in (first, second):
        writer.start(START_NS)
        fill(writer, 1)
        writer.stop()
        assert writer.finish(True).is_file()
    assert completed.read_bytes() == b'completed'
    assert interrupted.read_bytes() == b'interrupted'
    assert set(directory.glob('*.h5')) == {
        completed, interrupted, first.final_path, second.final_path,
    }


def test_appends_outside_episode_are_ignored(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    assert writer.append_arms(START_NS, np.zeros(14)) is False
    assert writer.append_rgb('top', START_NS, frame(0)) is False
    # ignored calls are not validated either: a late or early sampler cannot fail the writer
    assert writer.append_rgb('right_wrist', START_NS, frame(0)) is False
    assert writer.append_arms(START_NS, np.zeros(13)) is False
    assert writer.append_arms('stamp', np.zeros(14)) is False
    writer.check()

    writer.start(START_NS)
    assert writer.append_arms(START_NS, np.zeros(14)) is True
    assert writer.append_hands(START_NS, np.zeros(40)) is True
    assert writer.append_rgb('top', START_NS, frame(0)) is True
    assert writer.append_rgb('left_wrist', START_NS, frame(0)) is True
    writer.stop()
    assert writer.append_arms(START_NS + 1, np.zeros(14)) is False
    assert writer.append_hands(START_NS + 1, np.zeros(40)) is False
    assert writer.append_rgb('top', START_NS + 1, frame(0)) is False
    assert writer.finish(True).is_file()
    report = validate_episode(writer.final_path, config)
    assert report['counts']['arms'] == 1
    assert report['counts']['images'] == {'top': 1, 'left_wrist': 1}


def test_timestamp_rules_keep_ties_and_drop_pre_episode_cache(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    writer.start(START_NS)
    assert writer.append_hands(START_NS, np.zeros(40)) is True
    assert writer.append_rgb('top', START_NS, frame(0)) is True
    assert writer.append_rgb('left_wrist', START_NS, frame(0)) is True

    assert writer.append_arms(START_NS, np.zeros(14)) is True
    assert writer.append_arms(START_NS, np.ones(14)) is True  # duplicate stamp: both are stored
    writer.append_arms(START_NS - 1, np.full(14, 9.0))  # dropped by the writer: before start_ns
    assert writer.append_arms(START_NS + 10, np.full(14, 2.0)) is True

    writer.stop()
    final_path = writer.finish(True)
    assert writer.rejected_counts == {'arms': 1}
    with h5py.File(final_path, 'r') as episode:
        stamps = episode['observations/arms/timestamp_ns'][()]
        values = episode['observations/arms/qpos'][()]
    assert list(stamps) == [0, 0, 10]
    np.testing.assert_allclose(values[1], np.ones(14))
    assert values[:, 0].tolist() == [0.0, 1.0, 2.0]
    assert validate_episode(final_path, config)['counts']['arms'] == 3


def test_timestamp_regression_cannot_be_published_as_a_complete_episode(tmp_path):
    writer = EpisodeWriter(tmp_path, make_config(), TASK)
    writer.start(START_NS)
    writer.append_arms(START_NS + 10, np.zeros(14))
    writer.append_arms(START_NS + 5, np.ones(14))
    writer.stop()
    with pytest.raises(RuntimeError):
        writer.finish(True)
    writer.abort()
    assert writer.partial_path.exists()
    assert not writer.final_path.exists()


def test_invalid_samples_latch_fatal_and_preserve_partial(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    writer.start(START_NS)
    assert writer.append_hands(START_NS, np.zeros(40)) is True

    assert writer.append_arms(START_NS + 1, np.zeros(13)) is False
    with pytest.raises(RuntimeError, match='shape'):
        writer.check()
    assert writer.append_hands(START_NS + 1, np.full(40, np.nan)) is False
    assert writer.append_rgb('right_wrist', START_NS + 1, frame(0)) is False
    assert writer.append_rgb('top', START_NS + 1, np.zeros((HEIGHT, WIDTH), np.uint8)) is False

    writer.stop()
    with pytest.raises(RuntimeError, match='episode writer failed'):
        writer.finish(True)
    partial = writer.abort()
    assert partial.is_file()
    assert not writer.final_path.exists()
    with h5py.File(partial, 'r') as episode:
        assert episode['observations/hands/qpos'].shape == (1, 40)
        assert episode['observations/arms/qpos'].shape == (0, 14)


def test_queue_overflow_is_visible_and_fatal(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    gate = dataset.threading.Event()

    def blocked(self: EpisodeWriter, item: tuple) -> None:
        gate.wait(10)

    monkeypatch.setattr(EpisodeWriter, '_handle', blocked)
    writer = EpisodeWriter(tmp_path, make_config(), TASK, queue_capacity=4)
    writer.start(START_NS)
    accepted = sum(
        writer.append_arms(START_NS + index, np.zeros(14, np.float32)) for index in range(4 + 8)
    )
    assert accepted < 12  # the queue is bounded: capture must not silently drop
    with pytest.raises(RuntimeError, match='overflow'):
        writer.check()
    gate.set()
    writer.stop()
    with pytest.raises(RuntimeError, match='overflow'):
        writer.finish(True)
    assert writer.abort().is_file()


def test_writer_failure_is_visible(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def explode(self: EpisodeWriter, item: tuple) -> None:
        raise RuntimeError('disk full')

    monkeypatch.setattr(EpisodeWriter, '_handle', explode)
    writer = EpisodeWriter(tmp_path, make_config(), TASK)
    writer.start(START_NS)
    assert writer.append_arms(START_NS, np.zeros(14)) is True
    writer.stop()
    with pytest.raises(RuntimeError, match='disk full'):
        writer.finish(True)
    with pytest.raises(RuntimeError, match='disk full'):
        writer.check()
    partial = writer.abort()
    assert partial.is_file()
    assert not writer.final_path.exists()


def test_constructor_reports_initialization_failure(tmp_path: Path) -> None:
    blocked = tmp_path / 'not-a-directory'
    blocked.write_text('file', encoding='utf-8')
    with pytest.raises(RuntimeError, match='episode writer failed'):
        EpisodeWriter(blocked, make_config(), TASK)
    with pytest.raises(ValueError, match='task'):
        EpisodeWriter(tmp_path, make_config(), '')


@pytest.mark.parametrize('interruption', ['timeout', 'before_ready', 'after_ready'])
def test_unsuccessful_constructor_eventually_cleans_its_partial(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, interruption: str,
) -> None:
    now = datetime(2026, 9, 13, 12, 34, 56, 123456)
    monkeypatch.setattr(dataset.time, 'time_ns', lambda: int(now.timestamp() * 1e9))
    monkeypatch.setattr(dataset, '_TAKE_NUMBERS', count(1))
    monkeypatch.setattr(dataset, '_OPEN_TIMEOUT_S', 0.02)
    directory = tmp_path / '20260913'
    directory.mkdir()
    completed = directory / '20260913_123456_123456_take001.h5'
    existing_partial = directory / '20260913_123456_123456_take002.partial.h5'
    completed.write_bytes(b'completed')
    existing_partial.write_bytes(b'existing partial')

    opening = dataset.threading.Event()
    release = dataset.threading.Event()
    opened = []
    original_file = h5py.File
    original_wait = dataset.threading.Event.wait
    writer = EpisodeWriter.__new__(EpisodeWriter)

    def delayed_file(*args, **kwargs):
        opening.set()
        assert release.wait(5)
        episode = original_file(*args, **kwargs)
        opened.append(episode)
        return episode

    def interrupted_wait(event, timeout=None):
        if event is getattr(writer, '_ready', None) and interruption != 'timeout':
            if interruption == 'before_ready':
                assert original_wait(opening, 2)
            else:
                assert original_wait(event, 2)
            raise KeyboardInterrupt
        return original_wait(event, timeout)

    monkeypatch.setattr(dataset.h5py, 'File', delayed_file)
    monkeypatch.setattr(dataset.threading.Event, 'wait', interrupted_wait)
    if interruption == 'after_ready':
        release.set()
    try:
        expected_error = RuntimeError if interruption == 'timeout' else KeyboardInterrupt
        with pytest.raises(expected_error):
            writer.__init__(tmp_path, make_config(), TASK)
        assert opening.wait(2)
        if interruption != 'after_ready':
            assert writer._thread.is_alive()
            assert writer.partial_path.exists()
    finally:
        release.set()
        writer._thread.join(2)

    assert not writer._thread.is_alive()
    assert len(opened) == 1
    assert not opened[0].id.valid
    assert not writer.partial_path.exists()
    assert completed.read_bytes() == b'completed'
    assert existing_partial.read_bytes() == b'existing partial'


@pytest.mark.parametrize('queue_capacity', [1, 2])
def test_abort_timeout_preserves_partial_and_can_retry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, queue_capacity: int,
) -> None:
    handling = dataset.threading.Event()
    release = dataset.threading.Event()
    original_handle = EpisodeWriter._handle

    def delayed_handle(self: EpisodeWriter, item: tuple) -> None:
        handling.set()
        assert release.wait(5)
        original_handle(self, item)

    monkeypatch.setattr(EpisodeWriter, '_handle', delayed_handle)
    monkeypatch.setattr(dataset, '_CLOSE_TIMEOUT_S', 0.02)
    writer = EpisodeWriter(tmp_path, make_config(), TASK, queue_capacity=queue_capacity)
    writer.start(START_NS)
    try:
        assert writer.append_arms(START_NS, np.zeros(14))
        assert handling.wait(2)
        assert writer.append_arms(START_NS + 1, np.ones(14))
        for _ in range(2):
            with pytest.raises(RuntimeError, match='did not close'):
                writer.abort()
        with pytest.raises(RuntimeError, match='did not close'):
            writer.check()
        assert writer.partial_path.exists()
        assert writer._thread.is_alive()
        assert not writer.final_path.exists()
    finally:
        release.set()
        monkeypatch.setattr(dataset, '_CLOSE_TIMEOUT_S', 2)
        partial = writer.abort()
        writer._thread.join(2)

    assert not writer._thread.is_alive()
    assert writer.abort() == partial
    with pytest.raises(RuntimeError, match='did not close'):
        writer.check()
    with h5py.File(partial, 'r') as episode:
        np.testing.assert_array_equal(episode['observations/arms/timestamp_ns'][:], [0, 1])
        np.testing.assert_array_equal(
            episode['observations/arms/qpos'][:], [np.zeros(14), np.ones(14)],
        )
        assert 'success' not in episode.attrs


def test_abort_timeout_keeps_capture_error_and_handles_failed_worker_with_full_queue(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    handling = dataset.threading.Event()
    release = dataset.threading.Event()

    def failed_handle(self: EpisodeWriter, item: tuple) -> None:
        handling.set()
        assert release.wait(5)
        raise OSError('disk full')

    monkeypatch.setattr(EpisodeWriter, '_handle', failed_handle)
    monkeypatch.setattr(dataset, '_CLOSE_TIMEOUT_S', 0.02)
    writer = EpisodeWriter(tmp_path, make_config(), TASK, queue_capacity=1)
    writer.start(START_NS)
    try:
        assert writer.append_arms(START_NS, np.zeros(14))
        assert handling.wait(2)
        assert writer.append_arms(START_NS + 1, np.zeros(14))
        assert not writer.append_hands(START_NS, np.zeros(39))
        with pytest.raises(RuntimeError, match='did not close'):
            writer.abort()
        with pytest.raises(RuntimeError, match='shape'):
            writer.check()
    finally:
        release.set()
        writer._thread.join(2)

    assert not writer._thread.is_alive()
    partial = writer.abort()
    assert partial.is_file()
    with pytest.raises(RuntimeError, match='shape'):
        writer.check()
    with h5py.File(partial, 'r') as episode:
        assert 'success' not in episode.attrs


def test_finish_refuses_undecodable_jpeg(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def junk(_suffix: str, _image: np.ndarray, _params: list) -> tuple:
        return True, np.zeros((4, 1), np.uint8)

    monkeypatch.setattr(dataset.cv2, 'imencode', junk)
    writer = EpisodeWriter(tmp_path, make_config(), TASK)
    writer.start(START_NS)
    fill(writer, 2)
    writer.stop()
    with pytest.raises(EpisodeValidationError, match='JPEG'):
        writer.finish(True)
    assert writer.partial_path.is_file()
    assert not writer.final_path.exists()
    writer.abort()


def test_validator_rejects_schema_violations(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    writer.start(START_NS)
    fill(writer, 3)
    writer.stop()
    final_path = writer.finish(True)

    def rejected(pattern: str) -> None:
        with pytest.raises(EpisodeValidationError, match=pattern):
            validate_episode(final_path, config)

    with h5py.File(final_path, 'r') as episode:
        assert int(episode['images/top/jpeg'][0][0]) == 0xFF
        saved_frame = bytes(episode['images/top/jpeg'][1])

    with h5py.File(final_path, 'r+') as episode:  # extra camera placeholder
        episode.create_group('images/right_wrist')
    rejected('right_wrist')

    with h5py.File(final_path, 'r+') as episode:  # action group no longer part of the schema
        del episode['images/right_wrist']
        actions = episode.create_group('actions')
        actions.create_dataset('timestamp_ns', data=np.zeros(3, np.int64))
        actions.create_dataset('qpos', data=np.zeros((3, 54), np.float32))
    with pytest.raises(EpisodeValidationError, match='actions'):
        validate_episode(final_path, config)
    with h5py.File(final_path, 'r+') as episode:
        del episode['actions']

    with h5py.File(final_path, 'r+') as episode:  # non-monotonic timestamps
        episode['observations/arms/timestamp_ns'][2] = 1
    rejected('non-decreasing')

    with h5py.File(final_path, 'r+') as episode:  # non-finite values
        episode['observations/arms/timestamp_ns'][2] = 16_666_666
        episode['observations/hands/qpos'][1, 0] = np.nan
    rejected('non-finite')

    with h5py.File(final_path, 'r+') as episode:  # undecodable frame
        episode['observations/hands/qpos'][1, 0] = 0.0
        episode['images/top/jpeg'][1] = np.frombuffer(b'not a jpeg', dtype=np.uint8)
    rejected('JPEG')

    with h5py.File(final_path, 'r+') as episode:  # joint dimension mismatch
        episode['images/top/jpeg'][1] = np.frombuffer(saved_frame, dtype=np.uint8)
        del episode['observations/arms/qpos']
        episode.create_dataset('observations/arms/qpos', data=np.zeros((3, 13), np.float32))
    rejected('float32')
    with pytest.raises(EpisodeValidationError, match='robot_config'):
        validate_episode(final_path, make_config(robot_config='other_robot'))

    with h5py.File(final_path, 'r+') as episode:  # missing success attribute
        del episode.attrs['success']
    rejected('success')
    with pytest.raises(EpisodeValidationError, match='not found'):
        validate_episode(tmp_path / 'episode_000099.h5', config)


def test_aborted_partial_keeps_data_without_success_attribute(tmp_path: Path) -> None:
    config = make_config()
    writer = EpisodeWriter(tmp_path, config, TASK)
    writer.start(START_NS)
    fill(writer, 2)
    writer.stop()
    partial = writer.abort()
    assert partial.is_file() and partial.name.endswith('.partial.h5')
    assert not writer.final_path.exists()
    report = validate_episode(partial, config, require_success=False)
    assert report['success'] is None
    assert report['counts']['images'] == {'top': 2, 'left_wrist': 2}
    with pytest.raises(EpisodeValidationError, match='success'):
        validate_episode(partial, config)


def test_config_round_trip_and_validation() -> None:
    config = make_config()
    assert normalize_dataset_config(config) == config
    with pytest.raises(ValueError, match='missing'):
        normalize_dataset_config({key: value for key, value in config.items() if key != 'camera_names'})
    with pytest.raises(ValueError, match='policy_rate_hz'):
        normalize_dataset_config(make_config(policy_rate_hz=60))
    with pytest.raises(ValueError, match='jpeg_quality'):
        normalize_dataset_config(make_config(jpeg_quality=0))
    raw = make_config(image_encoding='rgb', jpeg_quality=None)
    assert normalize_dataset_config(raw) == raw
    with pytest.raises(ValueError, match='jpeg_quality'):
        normalize_dataset_config(make_config(image_encoding='rgb'))
    with pytest.raises(ValueError, match='jpeg_quality'):
        normalize_dataset_config(make_config(jpeg_quality=None))
    with pytest.raises(ValueError, match='image_encoding'):
        normalize_dataset_config(make_config(image_encoding='png'))
    with pytest.raises(ValueError, match='unique'):
        normalize_dataset_config(make_config(camera_names=['top', 'top']))
    with pytest.raises(ValueError, match='unknown'):
        normalize_dataset_config(make_config(action_type='absolute_joint_position_reference'))


def test_load_dataset_config(tmp_path: Path) -> None:
    writer = EpisodeWriter(tmp_path, make_config(), TASK)
    writer.abort()
    assert load_dataset_config(tmp_path) == normalize_dataset_config(make_config())
    (tmp_path / DATASET_CONFIG_NAME).write_text('{not json', encoding='utf-8')
    with pytest.raises(ValueError, match='JSON'):
        load_dataset_config(tmp_path)
