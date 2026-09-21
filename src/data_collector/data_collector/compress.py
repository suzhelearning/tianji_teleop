"""Offline, resumable RGB/JPEG dataset conversion to fixed-quality JPEG (Q50)."""

from __future__ import annotations

import argparse
from contextlib import contextmanager, nullcontext
import fcntl
import hashlib
import os
from pathlib import Path
import stat
import sys
import tempfile
from typing import Iterator, Mapping, Any

import cv2
import h5py
import numpy as np

from .dataset import (
    DATASET_CONFIG_NAME,
    dataset_config_bytes,
    ensure_dataset_config,
    load_dataset_config,
    normalize_dataset_config,
    validate_episode,
)

from tianji_runtime import compressed_dir as COMPRESSED_DATASET
from tianji_runtime import dataset_dir as DATASET

DEFAULT_SOURCE = DATASET()
DEFAULT_DESTINATION = COMPRESSED_DATASET()
JPEG_QUALITY = 50
_SOURCE_SHA256 = '_compression_source_sha256'
_SOURCE_CONFIG_SHA256 = '_compression_source_config_sha256'
_SOURCE_PATH = '_compression_source_relative_path'
_PROVENANCE_KEYS = (_SOURCE_SHA256, _SOURCE_CONFIG_SHA256, _SOURCE_PATH)


@contextmanager
def _destination_lock(root: Path) -> Iterator[None]:
    # Keep this inode permanently: unlinking a flock file would permit two owners.
    fd = os.open(root / '.compress.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o644)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'compression destination is locked by another process: {root}') from exc
        yield
    finally:
        os.close(fd)


def _file_identity(info: os.stat_result) -> tuple[int, int, int, int, int]:
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns


def _assert_unchanged(source: Path, fd: int, original: os.stat_result) -> None:
    identity = _file_identity(original)
    if _file_identity(os.fstat(fd)) != identity or _file_identity(source.lstat()) != identity:
        raise RuntimeError(f'source changed during compression: {source}')


def _sha256(fd: int) -> str:
    digest = hashlib.sha256()
    while block := os.read(fd, 1024 * 1024):
        digest.update(block)
    return digest.hexdigest()


def _copy_attributes(source: Any, destination: Any) -> None:
    for name in source.attrs:
        destination.attrs.create(name, source.attrs[name], dtype=source.attrs.get_id(name).dtype)


def _write_jpeg_episode(
    source: h5py.File,
    temporary: Path,
    config: Mapping[str, Any],
    provenance: Mapping[str, str],
) -> None:
    with h5py.File(temporary, 'w') as output:
        _copy_attributes(source, output)
        for name, value in provenance.items():
            output.attrs[name] = value
        source.copy('observations', output, expand_soft=True, expand_external=True, expand_refs=True)
        images = output.create_group('images')
        _copy_attributes(source['images'], images)
        for camera in config['camera_names']:
            raw_group = source[f'images/{camera}']
            group = images.create_group(camera)
            _copy_attributes(raw_group, group)
            raw_group.copy('timestamp_ns', group)
            frames = raw_group[config['image_encoding']]
            jpeg = group.create_dataset(
                'jpeg', shape=(frames.shape[0],), maxshape=(None,),
                chunks=(32,), dtype=h5py.vlen_dtype(np.uint8),
            )
            _copy_attributes(frames, jpeg)
            for index in range(frames.shape[0]):
                if config['image_encoding'] == 'rgb':
                    bgr = cv2.cvtColor(frames[index], cv2.COLOR_RGB2BGR)
                else:
                    # OpenCV decodes JPEG directly to the BGR encoder input.
                    bgr = cv2.imdecode(frames[index], cv2.IMREAD_COLOR)
                    if bgr is None:
                        raise RuntimeError(f'JPEG decoding failed for {camera} frame {index}')
                success, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not success:
                    raise RuntimeError(f'JPEG encoding failed for {camera} frame {index}')
                jpeg[index] = encoded.reshape(-1)


def _destination_parent(root: Path, relative: Path) -> Path:
    parent = root
    for component in relative.parts[:-1]:
        parent = parent / component
        if parent.is_symlink():
            raise RuntimeError(f'destination directory must not be a symlink: {parent}')
        parent.mkdir(exist_ok=True)
    return parent


def _compress_episode(
    source: Path,
    destination_root: Path,
    relative: Path,
    source_config: Mapping[str, Any],
    destination_config: Mapping[str, Any],
) -> bool:
    """Return True for a new atomic publication, False for a verified existing one."""
    fd = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        original = os.fstat(fd)
        if not stat.S_ISREG(original.st_mode):
            raise RuntimeError(f'source is not a regular file: {source}')
        try:
            # Cooperates with HDF5 writers even if their library locking is disabled
            # for this reader. Never wait for an active episode to become finalized.
            fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'source is active or locked: {source}') from exc
        validate_episode(source, source_config, require_success=True)
        with h5py.File(source, 'r', locking=True) as raw:
            if any(name in raw.attrs for name in _PROVENANCE_KEYS):
                raise RuntimeError(f'source uses reserved compression provenance attributes: {source}')
            provenance = {
                _SOURCE_SHA256: _sha256(fd),
                _SOURCE_CONFIG_SHA256: hashlib.sha256(dataset_config_bytes(source_config)).hexdigest(),
                _SOURCE_PATH: relative.as_posix(),
            }
            _assert_unchanged(source, fd, original)
            parent = _destination_parent(destination_root, relative)
            target = parent / relative.name
            if target.is_symlink():
                raise RuntimeError(f'existing destination must not be a symlink: {target}')
            if target.exists():
                if not target.is_file():
                    raise RuntimeError(f'existing destination is not a regular file: {target}')
                validate_episode(target, destination_config, require_success=True)
                with h5py.File(target, 'r', locking=True) as existing:
                    if any(existing.attrs.get(name) != value for name, value in provenance.items()):
                        raise RuntimeError(f'existing output does not match source provenance: {target}')
                _assert_unchanged(source, fd, original)
                return False

            temporary_fd, name = tempfile.mkstemp(
                prefix=f'.{target.stem}.compress-', suffix='.partial.h5', dir=parent,
            )
            temporary = Path(name)
            os.close(temporary_fd)
            try:
                _write_jpeg_episode(raw, temporary, source_config, provenance)
                validate_episode(temporary, destination_config, require_success=True)
                with temporary.open('rb') as completed:
                    os.fsync(completed.fileno())
                _assert_unchanged(source, fd, original)
                # Same-filesystem hard-link publication is atomic and cannot replace
                # an existing file, unlike rename/replace. Only validated files appear.
                os.link(temporary, target)
                directory_fd = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            finally:
                # Only remove our unique temporary, never another invocation's work.
                temporary.unlink(missing_ok=True)
            return True
    finally:
        os.close(fd)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', type=Path, default=DEFAULT_SOURCE,
                        help=f'Date-dataset parent or single RGB/JPEG dataset root (default: {DEFAULT_SOURCE})')
    parser.add_argument('destination', nargs='?', type=Path,
                        help=f'Output root (default: {DEFAULT_DESTINATION}/YYYYMMDD_compressed)')
    parser.add_argument('--date', help='Compress only this YYYYMMDD folder under the source root')
    args = parser.parse_args(argv)
    converted = skipped = failed = 0
    interrupted = False
    try:
        source = args.source.expanduser().resolve(strict=True)
        destination = (args.destination or DEFAULT_DESTINATION).expanduser().resolve()
        if args.date is not None:
            from datetime import datetime
            if len(args.date) != 8 or not args.date.isascii() or not args.date.isdigit():
                raise ValueError('--date must be YYYYMMDD')
            datetime.strptime(args.date, '%Y%m%d')
            if not (source / args.date).is_dir() or (source / args.date).is_symlink():
                raise ValueError(f'source date directory not found or is a symlink: {source / args.date}')
        if source == destination or source in destination.parents or destination in source.parents:
            raise ValueError('source and destination roots must not overlap')

        single_dataset = (source / DATASET_CONFIG_NAME).exists()
        if single_dataset:
            datasets = [source]
            if args.destination is None and len(source.name) == 8 and source.name.isascii() and source.name.isdigit():
                destination = destination / f'{source.name}_compressed'
        elif args.date:
            datasets = [source / args.date]
        else:
            datasets = sorted(
                path for path in source.iterdir()
                if len(path.name) == 8 and path.name.isascii() and path.name.isdigit()
                and not path.is_symlink() and path.is_dir()
            )
            if not datasets:
                raise ValueError(f'no date datasets found under source: {source}')
        if destination.is_symlink():
            raise RuntimeError(f'destination directory must not be a symlink: {destination}')
        destination.mkdir(parents=True, exist_ok=True)
        with _destination_lock(destination):
            for dataset in datasets:
                try:
                    source_config = load_dataset_config(dataset)
                    if source_config['image_encoding'] == 'jpeg':
                        print(f"Source {dataset}: JPEG Q{source_config['jpeg_quality']} -> Q{JPEG_QUALITY}; "
                              'lossy re-encoding, source files remain unchanged.')
                    destination_config = normalize_dataset_config({
                        **source_config, 'image_encoding': 'jpeg', 'jpeg_quality': JPEG_QUALITY,
                    })
                    if single_dataset:
                        dataset_destination = destination
                    else:
                        name = dataset.name if args.destination is not None else f'{dataset.name}_compressed'
                        dataset_destination = destination / name
                        if dataset_destination.is_symlink():
                            raise RuntimeError(f'destination directory must not be a symlink: {dataset_destination}')
                        dataset_destination.mkdir(exist_ok=True)
                    # A single-date invocation must exclude an aggregate invocation
                    # targeting the same standalone output dataset.
                    with nullcontext() if single_dataset else _destination_lock(dataset_destination):
                        ensure_dataset_config(dataset_destination, destination_config)

                        def walk_error(error: OSError) -> None:
                            nonlocal failed
                            failed += 1
                            print(f'FAILED scanning source: {error}', file=sys.stderr)

                        scan_root = source / args.date if single_dataset and args.date else dataset
                        for directory, directories, names in os.walk(
                            scan_root, followlinks=False, onerror=walk_error,
                        ):
                            directories[:] = sorted(
                                name for name in directories if not (Path(directory) / name).is_symlink()
                            )
                            for name in sorted(names):
                                if not name.endswith('.h5') or name.endswith('.partial.h5'):
                                    continue
                                episode = Path(directory) / name
                                if episode.is_symlink():
                                    continue
                                relative = episode.relative_to(dataset)
                                display_path = (dataset_destination / relative).relative_to(destination)
                                try:
                                    changed = _compress_episode(
                                        episode, dataset_destination, relative, source_config, destination_config,
                                    )
                                except Exception as exc:
                                    failed += 1
                                    print(f'FAILED {display_path}: {exc}', file=sys.stderr)
                                    continue
                                if changed:
                                    converted += 1
                                    print(f'CONVERTED {display_path}')
                                else:
                                    skipped += 1
                                    print(f'SKIPPED {display_path} (verified source identity and JPEG output)')
                        if load_dataset_config(dataset) != source_config:
                            raise RuntimeError('source dataset configuration changed during compression')
                except Exception as exc:
                    failed += 1
                    print(f'FAILED dataset {dataset}: {exc}', file=sys.stderr)
    except KeyboardInterrupt:
        interrupted = True
        failed += 1
        print('FAILED: compression interrupted; rerun to resume safely', file=sys.stderr)
    except Exception as exc:
        failed += 1
        print(f'FAILED: {exc}', file=sys.stderr)
    print(f'Summary: converted={converted} skipped={skipped} failed={failed}')
    return 130 if interrupted else (1 if failed else 0)


if __name__ == '__main__':
    raise SystemExit(main())
