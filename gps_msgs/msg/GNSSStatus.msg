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
int8 GNSS_MODE_NOT_SEEN = 0 # No update sent yet
int8 GNSS_MODE_NO_FIX = 1 # No fix
int8 GNSS_MODE_2D = 2 # 2D lat/lon only
int8 GNSS_MODE_3D = 3 # Lat, lon, and altitude all valid
