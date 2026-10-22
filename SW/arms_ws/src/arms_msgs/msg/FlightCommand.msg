# Final vehicle-independent command emitted by arms_control.
std_msgs/Header header
float32 roll
float32 pitch
float32 throttle
float32 yaw
bool arm
bool manual_mode
# PX4/Betaflight의 self-level(Stabilized) 스틱 해석을 요청한다.
# manual_mode와 달리 조종 스틱 passthrough를 뜻하지 않는다. PN 자동유도가
# 목표 roll/pitch 각도를 보낼 때 사용한다.
bool stabilized_mode
bool rtl_request
