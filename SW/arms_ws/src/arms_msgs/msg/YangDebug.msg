std_msgs/Header header

# Delayed EKF state from Yang et al. (§IV). Published while guidance_mode=2
# after the first valid target detection has initialized the observer.
bool valid
float32 image_age_sec
float32 relative_position_north_m
float32 relative_position_east_m
float32 relative_position_down_m
float32 relative_velocity_north_mps
float32 relative_velocity_east_mps
float32 relative_velocity_down_mps
float32 image_x
float32 image_y
float32 gyro_bias_x_radps
float32 gyro_bias_y_radps
float32 gyro_bias_z_radps
float32 accel_bias_x_mps2
float32 accel_bias_y_mps2
float32 accel_bias_z_mps2
