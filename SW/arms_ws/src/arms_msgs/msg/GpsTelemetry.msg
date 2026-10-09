# CRSF GPS (0x02) telemetry snapshot. Published only when a GPS frame arrives.
std_msgs/Header header
float64 latitude_deg
float64 longitude_deg
float32 altitude_m
float32 ground_speed_mps
float32 ground_course_deg
uint8 satellites
# MAVLink GPS_FIX_TYPE numeric values. CRSF 0x02 carries no actual fix type,
# so this is inferred only from the satellite count: 1=NO_FIX, 2=2D, 3=3D.
uint8 mavlink_fix_type
bool fix_type_inferred
bool valid_fix
