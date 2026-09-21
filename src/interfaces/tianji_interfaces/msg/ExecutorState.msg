# Observable executor state. This grants nothing: it can never be used to
# enable motion, to authorize Home, or to bypass the interactive gate.

string boot_id
string session_id

# Strictly increasing per publisher; lets a subscriber detect gaps.
uint64 sequence

# Incremented every time the phase value below changes. A recorded episode
# carries the revision it was opened under, so a stale control action cannot be
# applied to a different phase.
uint64 phase_revision

# Host CLOCK_MONOTONIC nanoseconds at publication.
int64 published_monotonic_ns

# One of: real, dry_run, simulation. Only `real` may be collected.
string mode

# Existing MotionGate/StagedMotionGate phase value, forwarded unchanged
# (WAITING/ALIGNING/READY/TELEOP/HOMING/HOME_REACHED).
string phase

# Set when the executor is in a fault state; recording must stop.
bool faulted

# Human-readable detail; empty when nominal.
string detail
