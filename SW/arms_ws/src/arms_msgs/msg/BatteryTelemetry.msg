# Battery telemetry snapshot decoded from CRSF 0x08.
std_msgs/Header header
float32 voltage_v               # pack voltage [V]
float32 current_a               # current [A]
int32 capacity_used_mah         # consumed capacity [mAh]
int32 battery_remaining_pct     # remaining capacity [%]
uint8 cell_count                # auto-detected cell count; 0 when unknown
