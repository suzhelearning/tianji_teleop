uint16 SCHEMA_VERSION=1
uint8 ARMS_READY=1
uint8 RIGHT_HAND_READY=2
uint8 LEFT_HAND_READY=4

uint16 schema_version
string robot_config
string session_id
uint64 sequence
builtin_interfaces/Time stamp
uint8 ready_mask
string[54] joint_names
float64[54] position_rad
