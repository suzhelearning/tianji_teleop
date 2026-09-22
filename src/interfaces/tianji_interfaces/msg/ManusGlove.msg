# Same-host CLOCK_MONOTONIC source time captured at the SDK callback, not ROS publish time.
# valid=false revokes this source; raw_nodes must never be interpreted as a target then.
uint32 glove_id
string side
string session_id
string boot_id
uint64 sequence
int64 source_monotonic_ns
bool valid
ManusRawNode[] raw_nodes
