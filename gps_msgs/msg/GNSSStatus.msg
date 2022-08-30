std_msgs/Header header
int8 mode
# Satellites used in solution
uint16 num_satellites_used # Number of satellites
gps_msgs/GNSSSatellite[<=255] satellites_used
# Satellites visible but not used in solution
uint16 num_satellites_unused
gps_msgs/GNSSSatellite[<=255] satellites_unused

# Consants related to mode
int8 GNSS_MODE_UNKNOWN = -1
int8 GNSS_MODE_NO_FIX = 0
int8 GNSS_MODE_2D = 1
int8 GNSS_MODE_3D = 2

# Constants related to quality
int8 QUALITY_INVALID = -1
int8 QUALITY_NO_SIGNAL = 0
int8 QUALITY_SEARCHING = 1
int8 QUALITY_ACQUIRED = 2
int8 QUALITY_DETECTED = 3
int8 QUALITY_CODE_LOCKED = 4
int8 QUALITY_CODE_CARRIER_LOCKED = 5
