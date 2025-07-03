bool yaw_use_odometry # Use odometry data for yaw angle correction.
float32 yaw_imu_gain # Gain for yaw angle from IMU (vs odometry)
bool disable_motor # Disable motor drive commands to make sure that the motor does not move.
bool simulate_motor # Simulate motor movement without motor controller feedback
bool clamp_imu_yaw_stationary # Clamp IMU yaw when vehicle is stationary
bool use_uwb_pos # Use UWB positioning instead of RTK positioning

float32 gear_ratio
float32 wheel_diam
float32 motor_poles
float32 steering_max_angle_rad # = arctan(axist_distance / turn_radius_at_maximum_steering_angle)
float32 steering_center
float32 steering_range
float32 steering_ramp_time # Ramp time constant for the steering servo in seconds
# Distance between front and rear wheels in ackermann mode, distance between drive wheels
# in differential mode.
float32 axis_distance
float32 vesc_p_gain
float32 vesc_i_gain
float32 vesc_d_gain
float32 anglemin
float32 anglemax
#float32 angledegrees
float32 voltage_centre

