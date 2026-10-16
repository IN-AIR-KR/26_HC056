# Source-independent vehicle attitude telemetry.
# Real hardware: ELRS CRSF Attitude (0x1E). SITL: PX4 MAVLink ATTITUDE.
std_msgs/Header header
float32 roll_deg
float32 pitch_deg
float32 yaw_deg
