# MediaPipe-order 21 XYZ landmarks in meters. Identity/time are unchanged from acquisition.
# Right-handed MANUS VUH world coordinates (XFromViewer, Z-up), without reflection.
# Each side has its own source sequence and age. valid=false is not a rest pose.
uint32 glove_id
string side
string session_id
string boot_id
uint64 sequence
int64 source_monotonic_ns
bool valid
# Includes upstream and adapter invalidations, retained on subsequent valid frames.
uint64 revocation_generation
float64[63] points
