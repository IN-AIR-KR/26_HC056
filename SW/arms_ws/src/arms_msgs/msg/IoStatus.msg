# Flight-controller transport health. Operator input timeout is handled by control.
std_msgs/Header header
string source
bool transport_connected
bool vehicle_telemetry_connected
uint32 last_rx_age_ms
uint32 last_vehicle_telemetry_age_ms
string fault_reason
