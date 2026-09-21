"""Offline conversion safety and data fidelity regressions; no hardware imports."""

from __future__ import annotations

import fcntl
from pathlib import Path
import sys

import cv2
import h5py
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_collector import compress  # noqa: E402
from data_collector.dataset import ensure_dataset_config, load_dataset_config, validate_episode  # noqa: E402


def make_source(
    root: Path, name: str = 'day/episode.h5', *, success: bool = False,
    jpeg_quality: int | None = None, robot_config: str = 'tianji_wuji2_v1',
) -> Path:
    config = {
        'schema_version': 1,
        'robot_config': robot_config,
        'joint_names': [f'joint_{index}' for index in range(54)],
        'joint_unit': 'rad',
        'policy_rate_hz': 30,
        'camera_names': ['top'],
        'image_width': 16,
        'image_height': 16,
        'image_encoding': 'rgb' if jpeg_quality is None else 'jpeg',
        'decoded_color_order': 'RGB',
        'jpeg_quality': jpeg_quality,
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
        if jpeg_quality is None:
            camera.create_dataset('rgb', data=rgb, chunks=(1, 16, 16, 3), maxshape=(None, 16, 16, 3))
        else:
            frames = camera.create_dataset('jpeg', shape=(2,), dtype=h5py.vlen_dtype(np.uint8))
            for index, frame in enumerate(rgb):
                ok, encoded = cv2.imencode(
                    '.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
                )
                assert ok
                frames[index] = encoded.reshape(-1)
        episode.attrs['operator_note'] = 'preserve root metadata'

        def annotate(name: str, obj: h5py.Group | h5py.Dataset) -> None:
            obj.attrs['origin'] = name
            obj.attrs['calibration'] = np.array([1, 2, 3], dtype=np.int16)

        episode.visititems(annotate)
    return path


@pytest.mark.parametrize('jpeg_quality', [None, 90], ids=['raw-rgb', 'legacy-jpeg-q90'])
def test_preserves_rgb_order_observations_timestamps_and_all_attributes(
    tmp_path: Path, jpeg_quality: int | None
) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'jpeg'
    episode = make_source(source, jpeg_quality=jpeg_quality)
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
            if isinstance(obj, h5py.Dataset) and not name.endswith(('/rgb', '/jpeg')):
                assert result.dtype == obj.dtype
                np.testing.assert_array_equal(result[:], obj[:])

        raw.visititems(compare)
        for index in range(2):
            encoded = jpeg['images/top/jpeg'][index]
            decoded_rgb = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            if jpeg_quality is None:
                source_rgb = raw['images/top/rgb'][index]
            else:
                source_rgb = cv2.cvtColor(
                    cv2.imdecode(raw['images/top/jpeg'][index], cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB
                )
            np.testing.assert_allclose(decoded_rgb, source_rgb, atol=5)
            # JPEG quantization tables carry the actual Q50 output contract.
            _, expected_q50 = cv2.imencode(
                '.jpg', cv2.cvtColor(source_rgb, cv2.COLOR_RGB2BGR),
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


@pytest.mark.parametrize('explicit_destination', [False, True], ids=['default', 'explicit'])
def test_date_datasets_keep_independent_configs_and_resume(
    tmp_path: Path, monkeypatch, capsys, explicit_destination: bool,
) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    episodes = [
        make_source(source / '20260919', 'episode.h5'),
        make_source(source / '20260920', 'nested/episode.h5', robot_config='tianji_wuji2_v2'),
    ]
    originals = {path: path.read_bytes() for path in source.rglob('*') if path.is_file()}
    monkeypatch.setattr(compress, 'DEFAULT_SOURCE', source)
    monkeypatch.setattr(compress, 'DEFAULT_DESTINATION', destination)
    argv = [str(source), str(destination)] if explicit_destination else []
    assert compress.main(argv) == 0
    assert 'Summary: converted=2 skipped=0 failed=0' in capsys.readouterr().out
    assert not (source / 'dataset_config.json').exists()
    assert not (destination / 'dataset_config.json').exists()
    targets = []
    for episode in episodes:
        relative = episode.relative_to(source)
        day = relative.parts[0]
        output_root = destination / (day if explicit_destination else f'{day}_compressed')
        config = load_dataset_config(output_root)
        assert config == {
            **load_dataset_config(source / day), 'image_encoding': 'jpeg', 'jpeg_quality': 50,
        }
        target = output_root.joinpath(*relative.parts[1:])
        validate_episode(target, config)
        targets.append(target)
    before = {path: (path.read_bytes(), path.stat().st_ino, path.stat().st_mtime_ns) for path in targets}
    assert compress.main(argv) == 0
    assert 'Summary: converted=0 skipped=2 failed=0' in capsys.readouterr().out
    for path, identity in before.items():
        assert (path.read_bytes(), path.stat().st_ino, path.stat().st_mtime_ns) == identity
    for path, content in originals.items():
        assert path.read_bytes() == content


def test_date_filter_does_not_read_other_date_configs(tmp_path: Path, monkeypatch, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    episode = make_source(source / '20260920', 'episode.h5')
    unrelated = source / '20260919'
    unrelated.mkdir()
    (unrelated / 'dataset_config.json').write_text('not JSON', encoding='utf-8')
    (unrelated / 'episode.h5').write_bytes(b'not HDF5')
    monkeypatch.setattr(compress, 'DEFAULT_DESTINATION', destination)
    assert compress.main([str(source), '--date', '20260920']) == 0
    assert 'Summary: converted=1 skipped=0 failed=0' in capsys.readouterr().out
    output_root = destination / '20260920_compressed'
    validate_episode(output_root / episode.name, load_dataset_config(output_root))
    assert not (destination / '20260919_compressed').exists()
    assert not (destination / 'dataset_config.json').exists()


def test_bad_date_config_does_not_block_independent_dates(tmp_path: Path, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    broken = make_source(source / '20260919', 'episode.h5')
    original = broken.read_bytes()
    (broken.parent / 'dataset_config.json').write_text('not JSON', encoding='utf-8')
    good = make_source(source / '20260920', 'episode.h5')
    assert compress.main([str(source), str(destination)]) == 1
    result = capsys.readouterr()
    assert 'Summary: converted=1 skipped=0 failed=1' in result.out
    assert '20260919' in result.err
    output_root = destination / '20260920'
    validate_episode(output_root / good.name, load_dataset_config(output_root))
    assert not (destination / '20260919').exists()
    assert broken.read_bytes() == original


def test_single_date_and_aggregate_share_default_output_and_lock(tmp_path: Path, monkeypatch, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    episode = make_source(source / '20260920', 'episode.h5')
    monkeypatch.setattr(compress, 'DEFAULT_DESTINATION', destination)
    assert compress.main([str(episode.parent)]) == 0
    output_root = destination / '20260920_compressed'
    validate_episode(output_root / episode.name, load_dataset_config(output_root))
    assert compress.main([str(source)]) == 0
    assert 'Summary: converted=0 skipped=1 failed=0' in capsys.readouterr().out
    assert not (destination / 'dataset_config.json').exists()
    with compress._destination_lock(output_root):
        assert compress.main([str(source)]) == 1


def test_aggregate_excludes_symlink_dates_and_rejects_symlink_output(tmp_path: Path, monkeypatch) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    episode = make_source(source / '20260920', 'episode.h5')
    (source / '20260919').symlink_to(episode.parent, target_is_directory=True)
    external = tmp_path / 'external'
    external.mkdir()
    destination.mkdir()
    (destination / '20260920_compressed').symlink_to(external, target_is_directory=True)
    monkeypatch.setattr(compress, 'DEFAULT_DESTINATION', destination)
    assert compress.main([str(source)]) == 1
    assert list(external.iterdir()) == []
    assert not (destination / '20260919_compressed').exists()
    assert compress.main([str(source), '--date', '20260919']) == 1


def test_existing_date_config_mismatch_preserves_output_and_other_dates(tmp_path: Path, capsys) -> None:
    source, destination = tmp_path / 'raw', tmp_path / 'compressed'
    conflicting = make_source(source / '20260919', 'episode.h5', robot_config='tianji_wuji2_v2')
    good = make_source(source / '20260920', 'episode.h5')
    output_root = destination / '20260919'
    ensure_dataset_config(output_root, {
        **load_dataset_config(conflicting.parent), 'robot_config': 'tianji_wuji2_v1',
        'image_encoding': 'jpeg', 'jpeg_quality': 50,
    })
    original = (output_root / 'dataset_config.json').read_bytes()
    assert compress.main([str(source), str(destination)]) == 1
    assert 'Summary: converted=1 skipped=0 failed=1' in capsys.readouterr().out
    assert (output_root / 'dataset_config.json').read_bytes() == original
    assert not (output_root / conflicting.name).exists()
    validate_episode(destination / '20260920' / good.name, load_dataset_config(destination / '20260920'))
