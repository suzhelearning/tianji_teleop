"""Offline conversion safety and data fidelity regressions; no hardware imports."""

from __future__ import annotations

import fcntl
from pathlib import Path
import sys

import cv2
import h5py
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from data_collection import compress  # noqa: E402
from data_collection.dataset import ensure_dataset_config, load_dataset_config, validate_episode  # noqa: E402


def make_source(root: Path, name: str = 'day/episode.h5', *, success: bool = False) -> Path:
    config = {
        'schema_version': 1,
        'robot_config': 'tianji_wuji2_v1',
        'joint_names': [f'joint_{index}' for index in range(54)],
        'joint_unit': 'rad',
        'policy_rate_hz': 30,
        'camera_names': ['top'],
        'image_width': 16,
        'image_height': 16,
        'image_encoding': 'rgb',
        'decoded_color_order': 'RGB',
        'jpeg_quality': None,
    }
    ensure_dataset_config(root, config)
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, 'w') as episode:
        episode.attrs.update(schema_version=1, robot_config=config['robot_config'], task='pick_place', success=success)
        observations = episode.create_group('observations')
        for stream, dim in (('arms', 14), ('hands', 40)):
            group = observations.create_group(stream)
            group.create_dataset('timestamp_ns', data=np.array([0, 33_333_333], dtype=np.int64))
            group.create_dataset('qpos', data=np.arange(2 * dim, dtype=np.float32).reshape(2, dim) / 100)
        images = episode.create_group('images')
        camera = images.create_group('top')
        camera.create_dataset('timestamp_ns', data=np.array([7, 34_000_000], dtype=np.int64))
        rgb = np.zeros((2, 16, 16, 3), dtype=np.uint8)
        rgb[0, :, :, 0] = 240
        rgb[1, :, :, 2] = 240
        camera.create_dataset('rgb', data=rgb, chunks=(1, 16, 16, 3), maxshape=(None, 16, 16, 3))
        episode.attrs['operator_note'] = 'preserve root metadata'

        def annotate(name: str, obj: h5py.Group | h5py.Dataset) -> None:
            obj.attrs['origin'] = name
            obj.attrs['calibration'] = np.array([1, 2, 3], dtype=np.int16)

        episode.visititems(annotate)
    return path


def test_preserves_rgb_order_observations_timestamps_and_all_attributes(tmp_path: Path) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source)
    original = episode.read_bytes()
    original_config = (source / 'dataset_config.json').read_bytes()
    assert compress.main([str(source), str(destination)]) == 0
    target = destination / episode.relative_to(source)
    config = load_dataset_config(destination)
    assert config == {**load_dataset_config(source), 'image_encoding': 'jpeg', 'jpeg_quality': 50}
    assert validate_episode(target, config)['success'] is False
    assert episode.read_bytes() == original
    assert (source / 'dataset_config.json').read_bytes() == original_config

    with h5py.File(episode, 'r') as raw, h5py.File(target, 'r') as jpeg:
        for name in raw.attrs:
            np.testing.assert_array_equal(jpeg.attrs[name], raw.attrs[name])

        def compare(name: str, obj: h5py.Group | h5py.Dataset) -> None:
            target_name = name[:-3] + 'jpeg' if name.endswith('/rgb') else name
            result = jpeg[target_name]
            for attribute in obj.attrs:
                np.testing.assert_array_equal(result.attrs[attribute], obj.attrs[attribute])
                assert result.attrs.get_id(attribute).dtype == obj.attrs.get_id(attribute).dtype
            if isinstance(obj, h5py.Dataset) and not name.endswith('/rgb'):
                assert result.dtype == obj.dtype
                np.testing.assert_array_equal(result[:], obj[:])

        raw.visititems(compare)
        for index in range(2):
            encoded = jpeg['images/top/jpeg'][index]
            decoded_rgb = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            np.testing.assert_allclose(decoded_rgb, raw['images/top/rgb'][index], atol=5)
            _, expected_q50 = cv2.imencode(
                '.jpg', cv2.cvtColor(raw['images/top/rgb'][index], cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, 50],
            )
            np.testing.assert_array_equal(encoded, expected_q50.reshape(-1))


def test_rerun_verifies_identity_without_overwrite_and_collects_new_files(tmp_path: Path, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source)
    assert compress.main([str(source), str(destination)]) == 0
    target = destination / episode.relative_to(source)
    original, before = target.read_bytes(), target.stat()
    later = make_source(source, 'next_day/later.h5', success=True)
    assert compress.main([str(source), str(destination)]) == 0
    assert 'Summary: converted=1 skipped=1 failed=0' in capsys.readouterr().out
    assert target.read_bytes() == original
    assert (target.stat().st_ino, target.stat().st_mtime_ns) == (before.st_ino, before.st_mtime_ns)
    assert validate_episode(destination / later.relative_to(source), load_dataset_config(destination))['success'] is True
    with h5py.File(episode, 'r+') as raw:
        raw['observations/arms/qpos'][0, 0] = 0.75
    assert compress.main([str(source), str(destination)]) == 1
    assert 'does not match source provenance' in capsys.readouterr().err
    assert target.read_bytes() == original


def test_excludes_partial_files_and_symlinks(tmp_path: Path) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source)
    (source / 'unfinished.partial.h5').write_bytes(b'in progress, not valid HDF5')
    (source / 'linked.h5').symlink_to(episode)
    (source / 'linked_day').symlink_to(episode.parent, target_is_directory=True)
    assert compress.main([str(source), str(destination)]) == 0
    assert sorted(path.relative_to(destination).as_posix() for path in destination.rglob('*.h5')) == ['day/episode.h5']


@pytest.mark.parametrize('relationship', ['same', 'child', 'parent'])
def test_rejects_overlapping_roots_without_modifying_source(tmp_path: Path, relationship: str) -> None:
    source = tmp_path / 'raw'
    episode = make_source(source)
    original = episode.read_bytes()
    destination = {'same': source, 'child': source / 'jpeg', 'parent': tmp_path}[relationship]
    assert compress.main([str(source), str(destination)]) == 1
    assert episode.read_bytes() == original
    assert not (destination / '.compress.lock').exists()


def test_interruption_never_publishes_and_does_not_remove_other_temporary_files(tmp_path: Path, monkeypatch) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source)
    parent = destination / 'day'
    parent.mkdir(parents=True)
    stale = parent / '.episode.compress-other.partial.h5'
    stale.write_bytes(b'another invocation owns this')
    encode = compress.cv2.imencode

    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(compress.cv2, 'imencode', interrupt)
    assert compress.main([str(source), str(destination)]) == 130
    assert not (destination / episode.relative_to(source)).exists()
    assert list(parent.iterdir()) == [stale]
    assert stale.read_bytes() == b'another invocation owns this'
    monkeypatch.setattr(compress.cv2, 'imencode', encode)
    assert compress.main([str(source), str(destination)]) == 0
    validate_episode(destination / episode.relative_to(source), load_dataset_config(destination))
    assert stale.read_bytes() == b'another invocation owns this'


def test_locked_source_and_damaged_files_fail_but_independent_episode_converts(tmp_path: Path, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    locked = make_source(source, 'locked.h5')
    good = make_source(source, 'good.h5')
    (source / 'damaged.h5').write_bytes(b'not HDF5')
    with locked.open('rb') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert compress.main([str(source), str(destination)]) == 1
    assert 'Summary: converted=1 skipped=0 failed=2' in capsys.readouterr().out
    assert (destination / good.name).exists()
    assert not (destination / locked.name).exists()
    assert not (destination / 'damaged.h5').exists()


def test_existing_output_must_validate_and_destination_lock_excludes_concurrent_runs(tmp_path: Path) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source)
    destination.mkdir()
    with compress._destination_lock(destination):
        assert compress.main([str(source), str(destination)]) == 1
    assert compress.main([str(source), str(destination)]) == 0
    target = destination / episode.relative_to(source)
    with h5py.File(target, 'r+') as output:
        output['images/top/jpeg'][0] = np.array([1, 2, 3], dtype=np.uint8)
    damaged = target.read_bytes()
    assert compress.main([str(source), str(destination)]) == 1
    assert target.read_bytes() == damaged
