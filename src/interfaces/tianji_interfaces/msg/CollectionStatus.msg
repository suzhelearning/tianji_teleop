# Collector status. Published 10 Hz and immediately on every transition.
#
# This reports what the collector has *already done*, not what a request asked
# for: a service response is an enqueue acknowledgement, the outcome is here.

string session_id

# Host CLOCK_MONOTONIC nanoseconds at publication.
int64 published_monotonic_ns

# One of: IDLE, STARTING, RECORDING, SAVING, DISCARDING, ABORTING, FAILED, CLOSED.
string state

# Cameras verified and the episode writer is prepared (directory writable,
# date layout resolved). Does not imply feedback is flowing.
bool prepared

# `prepared` plus fresh measured feedback for every device.
bool inputs_ready

# Partial file of the episode currently being recorded; empty when none.
string active_path

# Path of the most recently published success file; empty when none yet.
string last_saved_path

# Non-empty when the collector failed or an episode was kept as partial.
string error
