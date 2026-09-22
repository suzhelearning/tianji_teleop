"""ROS-independent per-hand stream validation and source-preserving processing."""

from dataclasses import dataclass
import math
import time


IDENTITY_FIELDS = (
    "glove_id", "side", "session_id", "boot_id", "sequence", "source_monotonic_ns",
)


@dataclass(frozen=True)
class Source:
    glove_id: int
    side: str
    session_id: str
    boot_id: str
    sequence: int
    source_monotonic_ns: int

    @classmethod
    def from_message(cls, message):
        return cls(**{name: getattr(message, name) for name in IDENTITY_FIELDS})

    @property
    def identity(self):
        return (self.boot_id, self.session_id, self.glove_id)


@dataclass(frozen=True)
class Output:
    source: Source
    valid: bool
    values: object = None
    reason: str = ""


class HandStage:
    """Validate one side before/after work; never cache or refresh another side.

    Duplicate valid samples are dropped. Timeout/error invalidation is emitted
    once, retaining the last source timestamp/sequence, with no usable values.
    A new fresh session/device resets history; retired identities cannot return.
    Publisher GIDs bind an identity to its writer in addition to graph checks.
    """

    def __init__(self, side, boot_id, transform, *, reset=None,
                 max_source_age_s=0.25, clock=time.monotonic_ns):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if not math.isfinite(max_source_age_s) or max_source_age_s <= 0:
            raise ValueError("max_source_age_s must be finite and positive")
        self.side = side
        self.boot_id = boot_id
        self.transform = transform
        self.reset = reset
        self.clock = clock
        self.max_age_ns = int(max_source_age_s * 1_000_000_000)
        self.last = None
        self.writer = None
        self.retired = set()
        self.live = False
        self.invalid_sent = False

    def _reset(self):
        if self.reset is not None:
            self.reset()

    def _invalidate(self, reason, source=None):
        if self.invalid_sent:
            return None
        source = self.last if self.last is not None else source
        if source is None:
            return None
        if self.live:
            self._reset()
        self.live = False
        self.invalid_sent = True
        return Output(source, False, reason=reason)

    def _age_error(self, source):
        age = self.clock() - source.source_monotonic_ns
        if source.source_monotonic_ns <= 0 or age < 0:
            return "future or missing source timestamp"
        if age > self.max_age_ns:
            return "stale source"
        return ""

    def process(self, message, payload, *, publisher_count=1, writer=None):
        source = Source.from_message(message)
        if source.side != self.side:
            return self._invalidate("side/topic mismatch", source)
        if source.boot_id != self.boot_id or not source.session_id:
            return self._invalidate("foreign boot or missing session", source)
        if source.sequence <= 0 or source.glove_id <= 0:
            return self._invalidate("missing source sequence or glove ID", source)
        if publisher_count != 1:
            return self._invalidate("missing or ambiguous publisher", source)
        if not message.valid:
            return self._invalidate("source reports invalid", source)
        error = self._age_error(source)
        if error:
            return self._invalidate(error, source)
        if source.identity in self.retired:
            return self._invalidate("retired source identity", source)
        changed = self.last is None or source.identity != self.last.identity
        if not changed:
            if writer != self.writer:
                return self._invalidate("source identity changed publisher", source)
            if (source.sequence <= self.last.sequence
                    or source.source_monotonic_ns <= self.last.source_monotonic_ns):
                return None
        else:
            if self.last is not None:
                self.retired.add(self.last.identity)
            self._reset()
            self.writer = writer
        self.last = source
        self.invalid_sent = False
        try:
            values = self.transform(payload)
        except (ValueError, RuntimeError) as error:
            # step() may have advanced internal history even if its result failed.
            self._reset()
            self.live = False
            return self._invalidate(str(error))
        error = self._age_error(source)
        if error:
            self._reset()
            self.live = False
            return self._invalidate(error)
        self.live = True
        return Output(source, True, values)

    def poll(self, *, publisher_count=1):
        if publisher_count != 1:
            return self._invalidate("missing or ambiguous publisher")
        if self.last is not None:
            error = self._age_error(self.last)
            if error:
                return self._invalidate(error)
        return None
