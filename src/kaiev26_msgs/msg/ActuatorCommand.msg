uint8 DRIVE_DISABLED=0
uint8 DRIVE_THROTTLE=1
uint8 DRIVE_TORQUE=2
uint8 DRIVE_SPEED=3

uint8 GEAR_NEUTRAL=0
uint8 GEAR_FORWARD=1
uint8 GEAR_REVERSE=2

std_msgs/Header header

bool enable
float32 steering_target_rad

uint8 drive_mode
float32 throttle_cmd
float32 drive_torque_nm
float32 speed_target_mps

bool brake_engage

uint8 gear
