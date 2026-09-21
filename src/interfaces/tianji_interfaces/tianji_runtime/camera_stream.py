"""Pair RGB images with their driver metadata and reject unusable streams.

Both the camera monitor and the collector consume the same ``Image`` +
``Metadata`` pair from the official RealSense driver, so the pairing rules live
here once. The rules exist because the driver publishes the two topics from
different publishers and with independent scheduling: a naive "latest metadata"
approach silently pairs a frame with a neighbouring frame's metadata, which
would make a dataset that looks valid but is wrong.

The validator reports frames; it never converts or copies image data, and it
imports nothing from ROS so it can be unit-tested against plain records.
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass

# Bounded pairing window. Five frames is far more than the observed skew between
# the two topics (one or two frames) while still refusing to pair across a stall.
MAX_PENDING = 5

# A pair that has not completed within this long is treated as a dead stream.
PAIR_TIMEOUT_S = 2.0

# ROS header stamps may not move backwards, and a frame older than this is
# dropped rather than re-stamped.
HEADER_MAX_AGE_S = 2.0
HEADER_FUTURE_TOLERANCE_S = 0.005


class StreamFault(RuntimeError):
    """The stream is unusable; the current recording segment must end."""


@dataclass(frozen=True)
class PairedFrame:
    """One complete, ordered, structurally valid camera sample."""

    role: str
    sequence: int
    frame_id: str
    timestamp_ns: int
    rgb: object
    received_ns: int


@dataclass(frozen=True)
class ImageRecord:
    """Structural facts of one Image message, captured at callback time."""

    stamp_ns: int
    frame_id: str
    width: int
    height: int
    step: int
    encoding: str
    rgb: object


@dataclass(frozen=True)
class MetadataRecord:
    """The subset of driver metadata used to order and de-duplicate frames."""

    stamp_ns: int
    frame_id: str
    frame_number: int


def split_stamp(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


# A metadata document is small (a few hundred bytes); this only bounds the
# damage from a malformed or never-completing payload.
METADATA_JSON_LIMIT = 64 * 1024


class MetadataJsonParser:
    """Decode the complete JSON document published by RealSense 4.58.3."""

    def __init__(self, role: str, *, limit: int = METADATA_JSON_LIMIT) -> None:
        self._role = role
        self._limit = limit

    def accept(self, frame_id: str, stamp_ns: int, json_data: str) -> MetadataRecord:
        if len(json_data) > self._limit:
            raise StreamFault(f"camera {self._role}: metadata exceeds {self._limit} bytes")
        try:
            document = json.loads(json_data)
        except (json.JSONDecodeError, TypeError) as error:
            raise StreamFault(f"camera {self._role}: malformed metadata JSON") from error
        if not isinstance(document, dict):
            raise StreamFault(f"camera {self._role}: metadata is not a JSON object")
        number = document.get("frame_number")
        if not isinstance(number, int) or isinstance(number, bool):
            raise StreamFault(
                f"camera {self._role}: metadata has no integer frame_number")
        return MetadataRecord(stamp_ns=stamp_ns, frame_id=frame_id, frame_number=number)


class CameraStreamValidator:
    """Assemble ordered pairs for one camera role and detect real stream faults.

    ``accept_image`` and ``accept_metadata`` return a :class:`PairedFrame` when a
    pair completes and ``None`` otherwise. A genuine stream failure raises
    :class:`StreamFault`; the caller ends the current segment rather than
    attempting to repair it, because a driver restart or a source sequence
    rewind means the frames no longer belong to one continuous capture.
    """

    def __init__(self, role: str, *, width: int, height: int, channels: int = 3,
                 max_pending: int = MAX_PENDING, pair_timeout_s: float = PAIR_TIMEOUT_S):
        self.role = role
        self.width = int(width)
        self.height = int(height)
        self.channels = int(channels)
        self._max_pending = int(max_pending)
        self._pair_timeout_ns = int(pair_timeout_s * 1e9)
        self._images: deque[tuple[ImageRecord, int]] = deque()
        self._metadata: deque[tuple[MetadataRecord, int]] = deque()
        self._image_publisher: str | None = None
        self._metadata_publisher: str | None = None
        self._frame_number: int | None = None
        self._metadata_stamp: int | None = None
        self._header_stamp: int | None = None
        self._watermark: int | None = None
        self._oldest_image_ns: int | None = None
        self._last_pair_ns: int | None = None
        self.last_frame: PairedFrame | None = None
        self.fault: str | None = None
        self.dropped_unpaired = 0
        self.sequence_gaps = 0
        self.max_pair_interval_ns = 0
        self.dropped_watermark = 0

    # ------------------------------------------------------------------ metadata

    def note_image_publisher(self, gid: bytes) -> None:
        """Pin the Image publisher for this stream.

        A changed publisher means the driver restarted; its frame numbering is
        unrelated to what has already been recorded, so the stream is failed
        rather than silently continued.
        """
        if self._image_publisher is None:
            self._image_publisher = gid
        elif self._image_publisher != gid:
            raise StreamFault(f"camera {self.role}: Image publisher changed mid-stream")

    def note_metadata_publisher(self, gid: bytes) -> None:
        if self._metadata_publisher is None:
            self._metadata_publisher = gid
        elif self._metadata_publisher != gid:
            raise StreamFault(f"camera {self.role}: Metadata publisher changed mid-stream")

    @property
    def image_publisher(self) -> str | None:
        return self._image_publisher.hex() if self._image_publisher else None

    # --------------------------------------------------------------------- input

    def accept_image(self, image: ImageRecord, received_ns: int) -> PairedFrame | None:
        """Record one Image callback and try to complete a pair."""
        if image.encoding != "rgb8":
            raise StreamFault(f"camera {self.role}: encoding {image.encoding!r}, expected 'rgb8'")
        expected_step = self.width * self.channels
        if (image.width, image.height) != (self.width, self.height):
            raise StreamFault(
                f"camera {self.role}: {image.width}x{image.height}, "
                f"expected {self.width}x{self.height}")
        if image.step != expected_step:
            raise StreamFault(
                f"camera {self.role}: step {image.step}, expected {expected_step}")
        if image.stamp_ns <= 0:
            raise StreamFault(f"camera {self.role}: image has no header stamp")
        if self._header_stamp is not None and image.stamp_ns < self._header_stamp:
            # Clock jump or source rewind: re-stamping would disguise the gap.
            raise StreamFault(f"camera {self.role}: header stamp went backwards")
        self._header_stamp = image.stamp_ns

        self._images.append((image, received_ns))
        while len(self._images) > self._max_pending:
            self._images.popleft()
            self.dropped_unpaired += 1
        self._oldest_image_ns = self._images[0][1]
        return self._pair()

    def accept_metadata(self, metadata: MetadataRecord, received_ns: int) -> PairedFrame | None:
        """Record one Metadata callback and try to complete a pair."""
        if metadata.frame_number < 0:
            raise StreamFault(f"camera {self.role}: negative frame number")
        if self._metadata_stamp is not None and metadata.stamp_ns > self._metadata_stamp:
            if metadata.frame_number < self._frame_number:
                raise StreamFault(
                    f"camera {self.role}: frame number went backwards "
                    f"({metadata.frame_number} < {self._frame_number})")
            if metadata.frame_number == self._frame_number:
                return None  # A newer stamp cannot turn duplicate source data into a sample.
        if self._watermark is not None and metadata.frame_number <= self._watermark:
            self.dropped_watermark += 1
            return None
        if self._metadata_stamp is None or metadata.stamp_ns > self._metadata_stamp:
            self._metadata_stamp = metadata.stamp_ns
            self._frame_number = metadata.frame_number
        if any(record == metadata for record, _ in self._metadata):
            return None
        self._metadata.append((metadata, received_ns))
        while len(self._metadata) > self._max_pending:
            self._metadata.popleft()
            self.dropped_unpaired += 1
        return self._pair()

    # -------------------------------------------------------------------- pairing

    def _pair(self) -> PairedFrame | None:
        """Pair the earliest complete (frame_id, stamp) match, oldest first."""
        self._expire()
        by_key = {(m.frame_id, m.stamp_ns): (m, received) for m, received in self._metadata}
        # An image arriving first must remain pending until its metadata arrives.
        # Select the oldest complete pair, not simply the oldest image.
        for image, received_ns in tuple(self._images):
            key = (image.frame_id, image.stamp_ns)
            match = by_key.get(key)
            if match is None:
                continue
            metadata, metadata_received_ns = match
            if self._watermark is not None and metadata.frame_number <= self._watermark:
                # Late arrival for an already published frame: drop it and keep
                # the watermark, so hashes and series never move backwards.
                self._images.remove((image, received_ns))
                self._metadata.remove(match)
                self.dropped_watermark += 1
                continue
            self._images.remove((image, received_ns))
            self._metadata.remove(match)
            if self._watermark is not None:
                self.sequence_gaps += max(0, metadata.frame_number - self._watermark - 1)
            self._watermark = metadata.frame_number
            self._oldest_image_ns = self._images[0][1] if self._images else None
            stamp = time.monotonic_ns()
            if self._last_pair_ns is not None:
                self.max_pair_interval_ns = max(self.max_pair_interval_ns, stamp - self._last_pair_ns)
            self._last_pair_ns = stamp
            frame = PairedFrame(
                role=self.role,
                sequence=metadata.frame_number,
                frame_id=image.frame_id,
                timestamp_ns=stamp,
                rgb=image.rgb,
                received_ns=min(received_ns, metadata_received_ns),
            )
            self.last_frame = frame
            return frame
        return None

    def _expire(self) -> None:
        now = time.monotonic_ns()
        if self._oldest_image_ns is not None and now - self._oldest_image_ns > self._pair_timeout_ns:
            raise StreamFault(
                f"camera {self.role}: no Image/Metadata pair completed within "
                f"{self._pair_timeout_ns / 1e9:.1f}s")

    # --------------------------------------------------------------------- health

    def check_fresh(self, now_ns: int | None = None) -> None:
        """Raise when the stream has stopped producing complete pairs."""
        now = time.monotonic_ns() if now_ns is None else now_ns
        if self._last_pair_ns is None:
            raise StreamFault(f"camera {self.role}: no complete pair received yet")
        age = now - self._last_pair_ns
        if age > self._pair_timeout_ns:
            raise StreamFault(f"camera {self.role}: last complete pair is {age / 1e9:.3f}s old")
