float32 px # Meters
float32 py # Meters
float32 pz # Meters
float32 vx # Meters / second
float32 vy # Meters / second
float32 speed # Meters / second
float32 roll # Degrees
float32 pitch # Degrees
float32 yaw # Degrees
float32 yaw_imu
float32 roll_rate # Degrees / second
float32 pitch_rate # Degrees / second
float32 yaw_rate # Degrees / second
float32 q0
float32 q1
float32 q2
float32 q3

# Current gps position and time stamp
float32 px_gps
float32 py_gps
float32 pz_gps
int32_t gps_ms
int gps_fix_type
float32 gps_last_corr_diff

# Previous GPS position and time stamp
float32 px_gps_last
float32 py_gps_last
float32 pz_gps_last
int32_t gps_ms_last

float32 gps_corr_cnt
float32 gps_ang_corr_x_last_gps
float32 gps_ang_corr_y_last_gps
int32_t gps_ang_corr_last_gps_ms
float32 gps_ground_level

uint32_t gps_corr_time
uint32_t ultra_update_time

# Multirotor state
float32 tilt_roll_err
float32 tilt_pitch_err

float32 error_vx_last
float32 error_vy_last

float32 vel_corr_x_int
float32 vel_corr_y_int

float32 tilt_corr_x_int
float32 tilt_corr_y_int
