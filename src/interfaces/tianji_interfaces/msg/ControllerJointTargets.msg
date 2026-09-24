# Final committed model-reference targets from the existing DLS/Ruckig core.
# NOT feedback and NOT enable permission. All angles are radians.
# produced_monotonic_ns is the solve/export time, input_monotonic_ns the applied
# PICO adapter's timestamp; do not replace an old input age with solve time.
string boot_id
string session_id
uint64 sequence
int64 produced_monotonic_ns
int64 input_monotonic_ns
uint64 tracking_epoch
# Zero outside episode-relative mode or before a reference is prepared.
uint64 reference_id
# bit0 arms ready, bit1 right legacy hand ready, bit2 left legacy hand ready.
uint8 flags
float64[7] left_arm
float64[7] right_arm
# Existing exoskeleton/TJH2 path only; this is NOT the new Manus ROS integration.
float64[20] left_hand
float64[20] right_hand
