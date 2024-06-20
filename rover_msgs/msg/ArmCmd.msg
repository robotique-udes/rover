uint8 LINEAR = 0
uint8 J0 = 1
uint8 J1 = 2
uint8 J2 = 3
uint8 GRIPPER_MOTOR_R = 4
uint8 GRIPPER_MOTOR_L = 5
uint8 GRIPPEROC = 6

bool[7] enable
float32[7] target_position
float32[7] current_position
bool[7] close_loop
