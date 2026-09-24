"""Executor status for the headset video copy, never a motion/display acknowledgement."""
from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
import threading
import time
import unicodedata

from PIL import Image, ImageDraw, ImageFont

from tianji_runtime.constants import IMAGE_HEIGHT, IMAGE_WIDTH


STATE_TIMEOUT_NS = 500_000_000
FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
_PHASE_TEXT = {
    "WAITING": "等待控制器就绪",
    "ALIGNING": "正在对齐，请勿移动",
    "READY": "控制器就绪，等待操作",
    "TELEOP": "遥操作中",
    "HOMING": "正在回到初始位置，请勿移动",
    "HOME_REACHED": "已到初始位置，等待操作",
    "PREFLIGHT": "正在检查启动条件",
}


def sanitize_detail(text: str) -> str:
    """Keep intentional line breaks, not terminal escapes or invisible bidi controls."""
    text = str(text)[:512].replace("\r\n", "\n").replace("\r", "\n")
    return "\n".join(" ".join("".join(
        " " if unicodedata.category(char).startswith("C") else char
        for char in line).split()) for line in text.split("\n")).strip()


@dataclass(frozen=True)
class HudText:
    title: str
    detail: str
    alert: bool = False


class ExecutorHudState:
    """A bounded latest-state handoff; duplicate/replayed samples cannot renew its lease."""

    def __init__(self):
        self._lock = threading.Lock()
        self._identity = None
        self._retired = set()
        self._sequence = 0
        self._published_ns = 0
        self._received_ns = 0
        self._text = None

    def receive(self, message, *, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        published = message.published_monotonic_ns
        if (not message.boot_id or not message.session_id or message.sequence <= 0
                or published <= 0 or not 0 <= now_ns - published <= STATE_TIMEOUT_NS):
            return
        mode = {"real": "真机", "dry_run": "试运行", "simulation": "仿真"}.get(message.mode, "未知模式")
        detail = sanitize_detail(message.detail) or _PHASE_TEXT.get(message.phase, "控制状态未知，请勿操作")
        title = f"{mode} | " + ("故障：停止操作" if message.faulted else "控制状态")
        text = HudText(title, detail, bool(message.faulted))
        identity = (message.boot_id, message.session_id)
        with self._lock:
            if identity in self._retired or published <= self._published_ns:
                return
            if identity == self._identity and message.sequence <= self._sequence:
                return
            if self._identity is not None and identity != self._identity:
                self._retired.add(self._identity)
            self._identity = identity
            self._sequence = message.sequence
            self._published_ns = published
            self._received_ns = now_ns
            self._text = text

    def snapshot(self, *, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        with self._lock:
            if self._text is None:
                return HudText("等待控制器状态", "请等待控制器启动；视频不代表可以操作")
            if (not 0 <= now_ns - self._published_ns <= STATE_TIMEOUT_NS
                    or not 0 <= now_ns - self._received_ns <= STATE_TIMEOUT_NS):
                return HudText("控制状态丢失：停止操作", "状态已过期，请查看主机；不要继续遥操作", True)
            return self._text


class ExecutorHud:
    """Cache text rasterization; copy RGB only when the encoder admits a new frame."""

    def __init__(self):
        self.state = ExecutorHudState()
        try:
            # Debian/Ubuntu fonts-noto-cjk's collection index 2 is Simplified Chinese.
            self._font = ImageFont.truetype(str(FONT_PATH), 32, index=2,
                                           layout_engine=ImageFont.Layout.BASIC)
            if self._font.getname()[0] != "Noto Sans CJK SC":
                raise ValueError("font collection index 2 is not Noto Sans CJK SC")
        except (OSError, ValueError) as error:
            raise RuntimeError(
                f"Chinese headset HUD font unavailable: {FONT_PATH}; "
                "install fonts-noto-cjk and python3-pil before starting the bridge") from error
        missing = self._font.getmask("\U0010ffff")
        self._missing = (missing.size, bytes(missing))
        self._coverage = {}
        for char in "控制状态丢失停止操作等待真机故障仿试运行请勿移动?":
            if not self._supported(char):
                raise RuntimeError(f"Chinese headset HUD font lacks required glyph U+{ord(char):04X}")
        self._cached_text = None
        self._banner = b""
        # Diagnose missing/broken fonts before declaring the bridge ready.
        self._rasterize(self.state.snapshot())

    def _supported(self, char):
        if char not in self._coverage:
            mask = self._font.getmask(char)
            self._coverage[char] = (mask.size, bytes(mask)) != self._missing
            if not self._coverage[char]:
                logging.getLogger(__name__).warning(
                    "Headset HUD font lacks U+%04X; replacing with '?' (not font tofu)", ord(char))
        return self._coverage[char]

    def _lines(self, text, limit):
        lines = []
        line = ""
        used = 0.0
        for char in text:
            if char != "\n" and not self._supported(char):
                char = "?"
            width = 0 if char == "\n" else self._font.getlength(char)
            if char == "\n" or used + width > IMAGE_WIDTH - 24:
                lines.append(line)
                line, used = "", 0.0
                if len(lines) == limit:
                    lines[-1] = lines[-1][:-1] + "…"
                    return lines
                if char == "\n":
                    continue
            line += char
            used += width
        if line or not lines:
            lines.append(line)
        return lines

    def _rasterize(self, text):
        lines = self._lines(text.title, 1) + self._lines(text.detail, 3)
        banner = Image.new("RGB", (IMAGE_WIDTH, 16 + 42 * len(lines)),
                           (100, 8, 8) if text.alert else (12, 19, 28))
        draw = ImageDraw.Draw(banner)
        for index, line in enumerate(lines):
            draw.text((12, 6 + 42 * index), line, font=self._font,
                      fill=(255, 235, 90) if index == 0 else (255, 255, 255))
        self._banner = banner.tobytes()
        self._cached_text = text

    def render(self, data: bytes) -> bytearray:
        """Return a private encoder buffer. The caller's recorded/published RGB is untouched.

        Called by the single encoder worker, never by the DDS image callback.
        The identical rendered copy is subsequently scaled and split into both eyes.
        """
        if not isinstance(data, bytes) or len(data) != IMAGE_WIDTH * IMAGE_HEIGHT * 3:
            raise ValueError("HUD input must be immutable tightly packed top-camera RGB8")
        text = self.state.snapshot()
        if text != self._cached_text:
            self._rasterize(text)
        video = bytearray(data)
        video[:len(self._banner)] = self._banner
        return video
