# Atomic matched PICO input, after the existing calibrated geometry adapter.
# source_timestamp_ns is the PICO source clock, not the host monotonic clock.
# published_monotonic_ns is the adapter's host CLOCK_MONOTONIC publication time.
# Same-host/boot only. This message has no hardware authority.
string boot_id
string session_id
uint64 sequence
uint64 tracking_epoch
# Retained on later samples so a dropped invalidation cannot disappear.
uint64 revocation_generation
int64 source_timestamp_ns
int64 published_monotonic_ns
bool valid
geometry_msgs/Pose left_target
geometry_msgs/Pose right_target
geometry_msgs/Vector3 left_arm_direction
geometry_msgs/Vector3 right_arm_direction
bool left_arm_direction_valid
bool right_arm_direction_valid
geometry_msgs/Point[8] upper_limb_points
geometry_msgs/Quaternion[8] upper_limb_rotations
bool upper_limb_valid
bool upper_limb_rotations_valid
bool user_button_pressed
