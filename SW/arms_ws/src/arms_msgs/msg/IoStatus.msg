# Flight-controller transport health. Operator input timeout is handled by control.
std_msgs/Header header
string source
bool transport_connected
bool vehicle_telemetry_connected
uint32 last_rx_age_ms
uint32 last_vehicle_telemetry_age_ms
float32 tx_uart_write_hz
float32 rx_uart_valid_frame_hz
float32 rx_uart_byte_rate
float32 rx_uart_crc_error_hz
string fault_reason
