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

# Yang et al. §III physical guidance output. These are logged only in the
# current stage; no FlightCommand conversion or actuator command is applied.
bool guidance_valid
float32 z1
float32 z2_x
float32 z2_y
float32 z2_z
float32 desired_accel_north_mps2
float32 desired_accel_east_mps2
float32 desired_accel_down_mps2
float32 omega1_body_x_radps
float32 omega1_body_y_radps
float32 omega1_body_z_radps
float32 omega2_body_x_radps
float32 omega2_body_y_radps
float32 omega2_body_z_radps
float32 omega_body_x_radps
float32 omega_body_y_radps
float32 omega_body_z_radps
float32 desired_thrust_n
