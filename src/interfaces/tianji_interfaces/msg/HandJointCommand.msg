# Desired Hand2 angles, NOT measured feedback and NOT motion authorization.
# Fixed firmware order: thumb,index,middle,ring,pinky; S1-S4 within each finger.
# Source identity/time survive adaptation and solving unchanged. Localhost/same boot only.
# valid=false revokes the target; position_rad must not be executed in that state.
uint32 glove_id
string side
string session_id
string boot_id
uint64 sequence
int64 source_monotonic_ns
bool valid
string[20] joint_names
float64[20] position_rad
