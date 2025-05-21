uint8 ID_CAM_MAIN = 0
uint8 ID_CAM_ANTENNA = 1
uint8 ID_CAM_FRONT_SIDE = 2
uint8 ID_CAM_ARM_TOP = 3
uint8 ID_CAM_ARM_SIDE = 4
uint8 ID_CAM_MAX = 5 # Delimtiter

uint8 STATUS_NONE = 0
uint8 STATUS_OK = 1
uint8 STATUS_ERROR = 2 # Delimtiter
uint8 STATUS_NO_PING = 2
uint8 STATUS_NO_STREAM = 3
uint8 STATUS_MAX = 4 # Delimtiter

uint8 id_cam
bool power_on
uint8 status
float32 yaw_main
float32 pitch_main
float32 yaw_antenna
float32 pitch_antenna
