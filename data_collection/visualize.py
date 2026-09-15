"""Loopback-only, read-only browser for schema-v1 Tianji HDF5 episodes."""

from __future__ import annotations

import argparse
import json
import math
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from urllib.parse import parse_qs, urlsplit

import cv2
import h5py
import numpy as np

from tianji.paths import DATASET

DEFAULT_DATASET = str(DATASET)
STREAM_DIMS = {'arms': 14, 'hands': 40}


class ViewerError(Exception):
    """An error safe to report to the local viewer."""

    def __init__(self, message: str, status: int = HTTPStatus.UNPROCESSABLE_ENTITY):
        super().__init__(message)
        self.status = status


def _json_value(value, label: str):
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist(), label)
    if isinstance(value, np.generic):
        scalar = value.item()
        if isinstance(scalar, np.generic):
            raise ViewerError(f'{label}: unsupported extended-precision attribute')
        return _json_value(scalar, label)
    if isinstance(value, bytes):
        try:
            return value.decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ViewerError(f'{label}: attribute is not UTF-8') from exc
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    if isinstance(value, (list, tuple)):
        return [_json_value(item, label) for item in value]
    raise ViewerError(f'{label}: attribute is not finite JSON data')


def _component(value: str, label: str) -> str:
    if not value or value in {'.', '..'} or any(c in value for c in '/\\\x00'):
        raise ViewerError(f'Invalid {label}: expected one name component', HTTPStatus.BAD_REQUEST)
    return value


class EpisodeStore:
    def __init__(self, dataset_path: Path | str):
        selected = Path(dataset_path).expanduser().resolve(strict=True)
        if selected.is_dir():
            self.root = selected
            self.single_file = None
        elif selected.is_file() and selected.suffix == '.h5':
            self.root = selected.parent
            self.single_file = selected
        else:
            raise ValueError('Dataset path must be a directory or an .h5 file')

    def episode_path(self, episode_id: str) -> Path:
        parts = episode_id.split('/')
        if (not episode_id or '\\' in episode_id or '\x00' in episode_id
                or PurePosixPath(episode_id).is_absolute()
                or any(part in {'', '.', '..'} for part in parts)):
            raise ViewerError('Invalid episode id: expected a relative .h5 path', HTTPStatus.BAD_REQUEST)
        candidate = self.root.joinpath(*parts)
        if candidate.suffix != '.h5':
            raise ViewerError('Episode id must end in .h5', HTTPStatus.BAD_REQUEST)
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise ViewerError(f'{episode_id}: episode not found or inaccessible', HTTPStatus.NOT_FOUND) from exc
        if not resolved.is_relative_to(self.root) or (
            self.single_file is not None and resolved != self.single_file
        ):
            raise ViewerError('Episode path is outside the selected dataset', HTTPStatus.FORBIDDEN)
        if not resolved.is_file() or resolved.suffix != '.h5':
            raise ViewerError(f'{episode_id}: not an .h5 file', HTTPStatus.NOT_FOUND)
        return resolved

    def episodes(self) -> dict:
        if self.single_file is not None:
            candidates = [self.single_file]
        else:
            candidates = []

            def scan_error(error):
                raise ViewerError(f'Cannot list dataset: {error.strerror}', HTTPStatus.FORBIDDEN)

            # Do not follow directory symlinks (including cycles or external trees).
            for directory, _, filenames in os.walk(self.root, followlinks=False, onerror=scan_error):
                candidates.extend(Path(directory) / name for name in filenames if name.endswith('.h5'))
        episodes = []
        for candidate in candidates:
            episode_id = candidate.relative_to(self.root).as_posix()
            try:
                path = self.episode_path(episode_id)
                size = path.stat().st_size
            except (ViewerError, OSError):
                # A writer may rename a partial file during the listing. Never publish escapes.
                continue
            episodes.append({'id': episode_id, 'partial': path.name.endswith('.partial.h5'), 'bytes': size})
        episodes.sort(key=lambda item: item['id'])
        return {'episodes': episodes}

    @staticmethod
    def _child(parent: h5py.Group, name: str, kind, optional: bool = False):
        label = f'{parent.name.rstrip("/")}/{name}'
        link = parent.get(name, getlink=True)
        if link is None:
            if optional:
                return None
            raise ViewerError(f'{label}: missing {kind.__name__.lower()}')
        # External/soft links, virtual datasets and external raw storage could escape the file.
        if not isinstance(link, h5py.HardLink):
            raise ViewerError(f'{label}: linked external or indirect data is not supported')
        value = parent[name]
        if not isinstance(value, kind):
            raise ViewerError(f'{label}: expected {kind.__name__.lower()}')
        if isinstance(value, h5py.Dataset) and (value.is_virtual or value.external):
            raise ViewerError(f'{label}: external or virtual storage is not supported')
        return value

    def _timestamp_dataset(self, group: h5py.Group) -> h5py.Dataset:
        dataset = self._child(group, 'timestamp_ns', h5py.Dataset)
        if dataset.ndim != 1 or dataset.dtype.kind != 'i' or dataset.dtype.itemsize != 8:
            raise ViewerError(f'{dataset.name}: expected int64 [N] timestamps')
        return dataset

    @staticmethod
    def _timestamps(dataset: h5py.Dataset) -> np.ndarray:
        values = dataset[()]
        if values.size and (values[0] < 0 or np.any(values[1:] < values[:-1])):
            raise ViewerError(f'{dataset.name}: timestamps must be nonnegative and nondecreasing')
        return values

    def _joint_names(self, path: Path) -> dict:
        for directory in path.parents:
            config = directory / 'dataset_config.json'
            try:
                if config.is_symlink():
                    raise ViewerError('dataset_config.json must not be a symbolic link')
                text = config.read_text(encoding='utf-8')
            except FileNotFoundError:
                continue
            except (OSError, UnicodeError) as exc:
                raise ViewerError(f'Cannot read nearest dataset_config.json: {exc}') from exc
            try:
                raw = json.loads(text)
            except ValueError as exc:
                raise ViewerError(f'Invalid nearest dataset_config.json: {exc}') from exc
            names = raw.get('joint_names') if isinstance(raw, dict) else None
            if (not isinstance(names, list) or len(names) != 54
                    or not all(isinstance(name, str) and name.strip() for name in names)
                    or len(set(names)) != 54):
                raise ViewerError('Nearest dataset_config.json must contain 54 unique nonempty joint_names')
            return {'arms': names[:14], 'hands': names[14:]}
        return {name: [f'{name}[{index}] (unnamed)' for index in range(dim)]
                for name, dim in STREAM_DIMS.items()}

    def _camera(self, images: h5py.Group, name: str):
        _component(name, 'camera')
        group = self._child(images, name, h5py.Group)
        if not group.keys():
            return None
        timestamps = self._timestamp_dataset(group)
        encodings = set(group.keys()) & {'rgb', 'jpeg'}
        if len(encodings) != 1:
            raise ViewerError(f'{group.name}: expected exactly one RGB or JPEG image stream')
        encoding = encodings.pop()
        frames = self._child(group, encoding, h5py.Dataset)
        if encoding == 'jpeg':
            if frames.ndim != 1 or h5py.check_vlen_dtype(frames.dtype) != np.dtype('uint8'):
                raise ViewerError(f'{frames.name}: expected vlen uint8 [N] JPEG data')
        elif (frames.ndim != 4 or frames.dtype != np.uint8
              or frames.shape[-1] != 3 or min(frames.shape[1:3]) < 1):
            raise ViewerError(f'{frames.name}: expected uint8 [N,H,W,3] RGB data')
        if frames.shape[0] != timestamps.size:
            raise ViewerError(f'{group.name}: image count does not match timestamp count')
        return timestamps, frames, encoding

    def metadata(self, episode_id: str) -> dict:
        path = self.episode_path(episode_id)
        try:
            names = self._joint_names(path)
            # Default HDF5 locking remains enabled; a live writer can make this fail explicitly.
            with h5py.File(path, 'r') as episode:
                attrs = {key: _json_value(value, f'attribute {key}') for key, value in episode.attrs.items()}
                if 'schema_version' in attrs and attrs['schema_version'] != 1:
                    raise ViewerError(f'Unsupported schema_version {attrs["schema_version"]!r}; expected 1')
                result = {'id': episode_id, 'partial': path.name.endswith('.partial.h5'),
                          'attrs': attrs, 'duration': 0.0, 'streams': {}, 'cameras': []}
                observations = self._child(episode, 'observations', h5py.Group, optional=True)
                if observations is not None:
                    for name, dim in STREAM_DIMS.items():
                        group = self._child(observations, name, h5py.Group, optional=True)
                        if group is None or not group.keys():
                            continue
                        timestamps = self._timestamps(self._timestamp_dataset(group))
                        qpos = self._child(group, 'qpos', h5py.Dataset)
                        if (qpos.ndim != 2 or qpos.shape != (timestamps.size, dim)
                                or qpos.dtype.kind != 'f' or qpos.dtype.itemsize > 8):
                            raise ViewerError(f'{qpos.name}: expected float32/float64 [{timestamps.size},{dim}] qpos')
                        values = qpos[()]
                        if not np.isfinite(values).all():
                            raise ViewerError(f'{qpos.name}: qpos contains non-finite values')
                        seconds = timestamps.astype(np.float64) / 1e9
                        result['streams'][name] = {'timestamps': seconds.tolist(), 'qpos': values.tolist(),
                                                   'names': names[name]}
                        if seconds.size:
                            result['duration'] = max(result['duration'], float(seconds[-1]))
                images = self._child(episode, 'images', h5py.Group, optional=True)
                if images is not None:
                    for name in sorted(images.keys()):
                        camera = self._camera(images, name)
                        if camera is None:
                            result['cameras'].append({'name': name, 'timestamps': [], 'count': 0})
                            continue
                        timestamps = self._timestamps(camera[0])
                        seconds = timestamps.astype(np.float64) / 1e9
                        result['cameras'].append({'name': name, 'timestamps': seconds.tolist(),
                                                  'count': int(timestamps.size), 'encoding': camera[2]})
                        if seconds.size:
                            result['duration'] = max(result['duration'], float(seconds[-1]))
                return result
        except ViewerError as exc:
            raise ViewerError(f'{episode_id}: {exc}', exc.status) from exc
        except (OSError, ValueError, TypeError, RuntimeError, KeyError) as exc:
            raise ViewerError(f'{episode_id}: cannot read HDF5 metadata (file may be active, incomplete or damaged): {exc}') from exc

    def image(self, episode_id: str, camera_name: str, index: int) -> bytes:
        path = self.episode_path(episode_id)
        _component(camera_name, 'camera')
        if index < 0:
            raise ViewerError('Image index must be nonnegative', HTTPStatus.BAD_REQUEST)
        try:
            with h5py.File(path, 'r') as episode:
                images = self._child(episode, 'images', h5py.Group, optional=True)
                if images is None or camera_name not in images:
                    raise ViewerError(f'Camera {camera_name!r} not found', HTTPStatus.NOT_FOUND)
                camera = self._camera(images, camera_name)
                if camera is None or index >= camera[1].shape[0]:
                    raise ViewerError(f'Image index {index} is out of range for {camera_name!r}', HTTPStatus.NOT_FOUND)
                # Read one frame only. PNG transport preserves raw RGB pixels;
                # it never changes the stored episode or creates an encoded dataset.
                raw = camera[1][index]
                if camera[2] == 'rgb':
                    ok, encoded = cv2.imencode('.png', cv2.cvtColor(raw, cv2.COLOR_RGB2BGR),
                                               [cv2.IMWRITE_PNG_COMPRESSION, 1])
                    if not ok:
                        raise ViewerError(f'{camera_name}[{index}]: cannot render RGB frame')
                    return encoded.tobytes()
                if raw.ndim != 1 or raw.dtype != np.uint8 or raw.size < 4 or bytes(raw[:2]) != b'\xff\xd8':
                    raise ViewerError(f'{camera_name}[{index}]: invalid JPEG byte stream')
                return raw.tobytes()
        except ViewerError as exc:
            raise ViewerError(f'{episode_id}: {exc}', exc.status) from exc
        except (OSError, ValueError, TypeError, RuntimeError, KeyError) as exc:
            raise ViewerError(f'{episode_id}: cannot read image (file may be active, incomplete or damaged): {exc}') from exc


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], store: EpisodeStore):
        self.store = store
        super().__init__(address, ViewerHandler)


class ViewerHandler(BaseHTTPRequestHandler):
    server_version = 'TianjiViewer/1'

    def _respond(self, status: int, body: bytes, content_type: str):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' blob:; frame-ancestors 'none'")
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, status: int, value):
        self._respond(status, json.dumps(value, ensure_ascii=True, allow_nan=False).encode('utf-8'),
                      'application/json; charset=utf-8')

    def send_error(self, code, message=None, explain=None):
        self._json(code, {'error': message or HTTPStatus(code).phrase})

    @staticmethod
    def _params(query: str, expected: set[str]) -> dict[str, str]:
        try:
            values = parse_qs(query, keep_blank_values=True, strict_parsing=True, max_num_fields=8,
                              encoding='utf-8', errors='strict')
        except (ValueError, UnicodeError) as exc:
            raise ViewerError('Invalid query parameters', HTTPStatus.BAD_REQUEST) from exc
        if set(values) != expected or any(len(items) != 1 or not items[0] for items in values.values()):
            raise ViewerError(f'Expected exactly these query parameters: {", ".join(sorted(expected)) or "none"}',
                              HTTPStatus.BAD_REQUEST)
        return {key: items[0] for key, items in values.items()}

    def do_GET(self):
        try:
            target = urlsplit(self.path)
            if target.scheme or target.netloc or target.fragment:
                raise ViewerError('Invalid request target', HTTPStatus.BAD_REQUEST)
            if target.path == '/':
                self._params(target.query, set())
                body = Path(__file__).with_name('visualize.html').read_bytes()
                self._respond(HTTPStatus.OK, body, 'text/html; charset=utf-8')
            elif target.path == '/api/episodes':
                self._params(target.query, set())
                self._json(HTTPStatus.OK, self.server.store.episodes())
            elif target.path == '/api/episode':
                params = self._params(target.query, {'id'})
                self._json(HTTPStatus.OK, self.server.store.metadata(params['id']))
            elif target.path == '/api/image':
                params = self._params(target.query, {'id', 'camera', 'index'})
                if not params['index'].isascii() or not params['index'].isdigit() or len(params['index']) > 20:
                    raise ViewerError('Image index must be a nonnegative integer', HTTPStatus.BAD_REQUEST)
                body = self.server.store.image(params['id'], params['camera'], int(params['index']))
                content_type = 'image/png' if body.startswith(b'\x89PNG\r\n\x1a\n') else 'image/jpeg'
                self._respond(HTTPStatus.OK, body, content_type)
            else:
                raise ViewerError('Unknown endpoint', HTTPStatus.NOT_FOUND)
        except ViewerError as exc:
            self._json(exc.status, {'error': str(exc)})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.log_error('Request failed: %s', exc)
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {'error': 'Unable to serve request; see viewer terminal for details'})

    def _read_only(self):
        self._json(HTTPStatus.METHOD_NOT_ALLOWED, {'error': 'Read-only viewer: only GET is supported'})

    do_POST = do_PUT = do_PATCH = do_DELETE = do_HEAD = do_OPTIONS = _read_only


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Read-only local viewer for Tianji schema-v1 HDF5 data; no hardware access.')
    parser.add_argument('dataset_path', nargs='?', default=DEFAULT_DATASET,
                        help=f'Dataset directory or single .h5 file (default: {DEFAULT_DATASET})')
    parser.add_argument('--port', type=int, default=8765, help='Loopback HTTP port (default: 8765; 0 chooses a free port)')
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error('--port must be between 0 and 65535')
    try:
        store = EpisodeStore(args.dataset_path)
        server = ViewerServer(('127.0.0.1', args.port), store)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    with server:
        print(f'Tianji read-only viewer: http://127.0.0.1:{server.server_port}/', flush=True)
        print(f'Dataset: {store.single_file or store.root}', flush=True)
        print('No hardware access. Press Ctrl+C to stop.', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
