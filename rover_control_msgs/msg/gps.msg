# Constants
uint8 FIX_INVALID = 0
uint8 FIX_STANDALONE = 1
uint8 FIX_DGPS = 2
uint8 FIX_RTK_FIXED = 4
uint8 FIX_RTK_FLOAT = 5

# Members
float32 latitude
float32 longitude
float32 height
uint8 fix_quality
float32 heading
uint8 satellite
uint16 reference_station_ID 
