"""Live, camera-only desktop preview; no recording, ROS or robot connection."""
from contextlib import ExitStack
from pathlib import Path
import argparse
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from data_collection.config import load_collection_config
from data_collection.runtime import FPS, _RealSenseCamera, validate_camera_profiles

WINDOW = 'Tianji live cameras - Q / Esc to quit'
TILE_WIDTH, TILE_HEIGHT, HEADER = 640, 360, 64


def _receive_frames(source, stream, lock, stop):
    """Keep only the newest owned tile; desktop timing never gates SDK reads."""
    previous = None
    try:
        while not stop.is_set():
            frame = source.read(2000)
            if frame is None or frame[0] == previous:
                continue
            sequence, pixels = frame
            received = time.monotonic()
            tile = cv2.resize(pixels, (TILE_WIDTH, TILE_HEIGHT), interpolation=cv2.INTER_AREA)
            cv2.cvtColor(tile, cv2.COLOR_RGB2BGR, dst=tile)
            with lock:
                stream['frame'] = tile
                stream['count'] += 1
                stream['last'] = received
            previous = sequence
    except Exception as error:
        if not stop.is_set():
            with lock:
                stream['error'] = str(error)


def run(config_path):
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('No desktop display. Run this command in a desktop terminal.')
    _, cameras = load_collection_config(config_path)
    context = rs.context()
    profiles = validate_camera_profiles(cameras, realsense=rs, context=context)
    canvas = np.zeros((TILE_HEIGHT + HEADER, TILE_WIDTH * len(cameras), 3), dtype=np.uint8)
    stop, lock = threading.Event(), threading.Lock()
    with ExitStack() as cleanup:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cleanup.callback(cv2.destroyAllWindows)
        cv2.resizeWindow(WINDOW, min(canvas.shape[1], 1440),
                         round(canvas.shape[0] * min(canvas.shape[1], 1440) / canvas.shape[1]))
        streams = []
        for name, serial in cameras.items():
            try:
                source = _RealSenseCamera(rs, context, name, serial)
            except RuntimeError as error:
                raise RuntimeError(f'{name} SN={serial}: {error}. Stop collection.sh, '
                                   'realsense.sh or any other camera user before previewing.') from error
            cleanup.callback(source.close)
            stream = {'name': name, 'serial': serial, 'frame': None, 'last': None,
                      'count': 0, 'error': None, 'displayed': 0, 'shown': 0,
                      'rx_base': 0, 'ui_base': 0, 'rx_rate': 0., 'ui_rate': 0.}
            streams.append(stream)
            thread = threading.Thread(target=_receive_frames, args=(source, stream, lock, stop),
                                      name=f'preview-{name}')
            thread.start()
            # LIFO: request stop, join the reader, then release its SDK stream.
            cleanup.callback(thread.join)
            cleanup.callback(stop.set)
            print(profiles[name], flush=True)
        begin = logged = measured = time.monotonic()
        with lock:
            for stream in streams:
                stream['rx_base'] = stream['count']
        print('Camera-only preview; RX=received, UI=new frames shown (2s window). '
              'Q / Esc / window close / Ctrl+C: quit.', flush=True)
        while True:
            now = time.monotonic()
            measure = now - measured >= 2
            for index, stream in enumerate(streams):
                with lock:
                    tile, count, last, error = (stream[key] for key in ('frame', 'count', 'last', 'error'))
                if error:
                    raise RuntimeError(f"{stream['name']} SN={stream['serial']}: {error}")
                if (last is None and now - begin > 10) or (last is not None and now - last > 2):
                    raise RuntimeError(f"{stream['name']} SN={stream['serial']}: no new frames; stopping preview")
                x = index * TILE_WIDTH
                if tile is not None and count != stream['displayed']:
                    canvas[HEADER:, x:x + TILE_WIDTH] = tile
                    stream['displayed'] = count
                    stream['shown'] += 1
                if measure:
                    stream['rx_rate'] = (count - stream['rx_base']) / (now - measured)
                    stream['ui_rate'] = (stream['shown'] - stream['ui_base']) / (now - measured)
                    stream['rx_base'], stream['ui_base'] = count, stream['shown']
                canvas[:HEADER, x:x + TILE_WIDTH] = 0
                cv2.putText(canvas, f"{stream['name']} | {stream['serial']}",
                            (x + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 1, cv2.LINE_AA)
                status = (f"RX {stream['rx_rate']:.1f} | UI {stream['ui_rate']:.1f} FPS (2s)"
                          if now - begin >= 2 else 'measuring...')
                cv2.putText(canvas, status, (x + 10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                            .65, (255, 255, 255), 1, cv2.LINE_AA)
            if measure:
                measured = now
            cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if now - logged >= 5:
                print('LIVE ' + ' | '.join(
                    f"{s['name']}: RX={s['rx_rate']:.1f} UI={s['ui_rate']:.1f} FPS"
                    for s in streams), flush=True)
                logged = now
            # Service the desktop at up to 60 Hz, without busy-polling cameras.
            stop.wait(max(0., 1 / (2 * FPS) - (time.monotonic() - now)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'collection_config.json',
                        help='Camera roles and serials (default: collection_config.json)')
    args = parser.parse_args()

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        run(args.config)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError, cv2.error) as error:
        print(f'Live preview failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
