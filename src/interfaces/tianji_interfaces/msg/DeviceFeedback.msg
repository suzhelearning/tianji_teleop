# One measured device sample plus the identity needed to trust it.
#
# Payload, source identity and health are deliberately kept in a single sample:
# a consumer must never be able to pair a payload with a different session or a
# stale health flag. `boot_id`/`session_id`/`sequence` are checked by every
# consumer before a sample may reach a dataset.

# Host identity of the publisher (/proc/sys/kernel/random/boot_id). A sample
# whose boot_id differs from the subscriber's own is rejected, never rescaled.
string boot_id

# UUID created once when the executor process starts. Changes on every
# executor restart, so samples from a finished session can never extend a new one.
string session_id

# Strictly increasing per device, never reused or rewound within a session.
uint64 sequence

# Host CLOCK_MONOTONIC nanoseconds when this sample was published.
# This is the same clock as source_monotonic_ns, so
# (published_monotonic_ns - source_monotonic_ns) is a real age in nanoseconds.
int64 published_monotonic_ns

# Verbatim device clock stamp taken from the SDK read (Feedback.received_monotonic_ns).
# Never re-stamped by DDS send, by the other hand's update, or by the writer.
int64 source_monotonic_ns

# One of: arms, left_hand, right_hand.
string device

# Measured joint positions in radians: 14 for arms, 20 per hand.
float64[] position_rad

# Device reports a valid, in-range sample.
bool healthy

# Device reports servo output enabled; distinct from `healthy`.
bool enabled

# Human-readable reason when unhealthy or degraded; empty when nominal.
string detail
