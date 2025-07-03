# GPS position
float64 lat
float64 lon
float64 height
float64 x
float64 y
float64 z
int fix_type # 0=Invalid, 1=SPP, 4=RTK fix, 5=RTK float
int sats
int32_t ms # Milliseconds today
uint32_t update_time
# Local position (ENU frame)
bool local_init_done
float32 lx
float32 ly
float32 lz
# Initial local position
float64 ix
float64 iy
float64 iz
# Rotation matrix for local position
float32 r1c1, r1c2, r1c3
float32 r2c1, r2c2, r2c3
float32 r3c1, r3c2, r3c3

