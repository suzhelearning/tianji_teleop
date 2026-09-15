"""按 HDF5 相机时间戳同步显示 HEVC 录制视频。"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

from .timeline import frame_index_at_timestamp


class SynchronizedVideoPlayer:
    """用 FFmpeg 解码、OpenCV 显示的时间戳驱动视频窗口。"""

    def __init__(
        self,
        video_path: str | Path,
        timestamps_ns: np.ndarray,
        *,
        rotation_degrees_ccw: int,
        max_long_edge: int = 720,
        window_name: str = "Data glove camera replay",
        ffmpeg_binary: str | Path = "ffmpeg",
        window_x: int = 1400,
        window_y: int = 100,
    ) -> None:
        self.path = Path(video_path).expanduser().resolve()
        self.timestamps_ns = np.asarray(timestamps_ns, dtype=np.int64)
        self.rotation_degrees_ccw = int(rotation_degrees_ccw)
        self.max_long_edge = int(max_long_edge)
        self.window_name = str(window_name)
        executable = shutil.which(str(Path(ffmpeg_binary).expanduser()))
        if executable is None:
            raise FileNotFoundError(f"FFmpeg 可执行文件不存在或不可执行：{ffmpeg_binary}")
        self.ffmpeg_binary = Path(executable).resolve()
        self.window_x = int(window_x)
        self.window_y = int(window_y)
        if not self.path.is_file():
            raise FileNotFoundError(f"回放视频不存在：{self.path}")
        if self.timestamps_ns.ndim != 1 or self.timestamps_ns.size == 0:
            raise ValueError("视频时间戳必须是非空一维数组")
        if self.rotation_degrees_ccw not in (0, 90, 180, 270):
            raise ValueError("视频旋转必须是 0/90/180/270")
        if self.max_long_edge <= 0:
            raise ValueError("视频显示长边必须是正整数")
        if self.window_x < 0 or self.window_y < 0:
            raise ValueError("视频窗口位置不能为负")
        width, height, frame_count = _probe_video(self.path)
        if frame_count != int(self.timestamps_ns.size):
            raise ValueError(
                "MP4 帧数与 HDF5 相机时间戳不一致："
                f"video={frame_count}，timestamps={self.timestamps_ns.size}"
            )
        scale = min(1.0, self.max_long_edge / max(width, height))
        self.decode_width = _even_dimension(width * scale)
        self.decode_height = _even_dimension(height * scale)
        self._process: subprocess.Popen[bytes] | None = None
        self._next_decode_index = 0
        self._last_shown_index: int | None = None
        self._loop_count: int | None = None
        self._window_created = False

    def show_at(self, timestamp_ns: int, *, loop_count: int) -> bool:
        """显示不晚于录制时刻的最新视频帧。"""

        target = frame_index_at_timestamp(self.timestamps_ns, timestamp_ns)
        if target is None:
            return self._pump_window()
        if self._loop_count != int(loop_count) or (
            self._last_shown_index is not None
            and target < self._last_shown_index
        ):
            self._restart_decoder()
            self._loop_count = int(loop_count)
        if target == self._last_shown_index:
            return self._pump_window()
        frame: np.ndarray | None = None
        while self._next_decode_index <= target:
            frame = self._read_frame()
            self._next_decode_index += 1
        if frame is None:
            return self._pump_window()
        displayed = np.rot90(
            frame,
            k=self.rotation_degrees_ccw // 90,
        )
        if not self._window_created:
            cv2.namedWindow(
                self.window_name,
                cv2.WINDOW_NORMAL
                | cv2.WINDOW_KEEPRATIO
                | cv2.WINDOW_GUI_NORMAL,
            )
            cv2.resizeWindow(
                self.window_name,
                int(displayed.shape[1]),
                int(displayed.shape[0]),
            )
            cv2.moveWindow(self.window_name, self.window_x, self.window_y)
            self._window_created = True
        cv2.imshow(self.window_name, np.ascontiguousarray(displayed))
        self._last_shown_index = target
        return self._pump_window()

    def _read_frame(self) -> np.ndarray:
        if self._process is None:
            self._start_decoder()
        process = self._process
        if process is None or process.stdout is None:
            raise RuntimeError("FFmpeg 解码器未启动")
        expected = self.decode_width * self.decode_height * 3
        chunks = bytearray()
        while len(chunks) < expected:
            chunk = process.stdout.read(expected - len(chunks))
            if not chunk:
                status = process.poll()
                raise RuntimeError(
                    "FFmpeg 视频帧提前结束："
                    f"index={self._next_decode_index}，exit={status}"
                )
            chunks.extend(chunk)
        return np.frombuffer(chunks, dtype=np.uint8).reshape(
            self.decode_height,
            self.decode_width,
            3,
        )

    def _start_decoder(self) -> None:
        command = [
            str(self.ffmpeg_binary),
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-hwaccel",
            "auto",
            "-i",
            str(self.path),
            "-an",
            "-vf",
            (
                f"scale={self.decode_width}:{self.decode_height},"
                "format=bgr24"
            ),
            "-vsync",
            "0",
            "-f",
            "rawvideo",
            "pipe:1",
        ]
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def _restart_decoder(self) -> None:
        self._stop_decoder()
        self._next_decode_index = 0
        self._last_shown_index = None

    def _stop_decoder(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.stdout is not None:
            process.stdout.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        else:
            process.wait()

    def _pump_window(self) -> bool:
        cv2.waitKey(1)
        if not self._window_created:
            return True
        return cv2.getWindowProperty(
            self.window_name,
            cv2.WND_PROP_VISIBLE,
        ) >= 1.0

    def close(self) -> None:
        self._stop_decoder()
        if self._window_created:
            cv2.destroyWindow(self.window_name)
            cv2.waitKey(1)
            self._window_created = False

    def __enter__(self) -> "SynchronizedVideoPlayer":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


def _probe_video(path: Path) -> tuple[int, int, int]:
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise ValueError(f"无法打开回放视频：{path}")
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    finally:
        capture.release()
    if width <= 0 or height <= 0 or frame_count <= 0:
        raise ValueError(
            f"回放视频尺寸/帧数无效：{width}x{height}/{frame_count}"
        )
    return width, height, frame_count


def _even_dimension(value: float) -> int:
    rounded = max(2, int(round(float(value))))
    return rounded if rounded % 2 == 0 else rounded + 1
