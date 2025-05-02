# Operating mode of receiver
int32 GNSS_MODE_NOT_SEEN=0
int32 GNSS_MODE_NO_FIX=1
int32 GNSS_MODE_2D=2
int32 GNSS_MODE_3D=3

int32 mode 0

# Fix status
int32 GNSS_STATUS_NO_FIX=0
int32 GNSS_STATUS_DGPS_FIX=1
int32 GNSS_STATUS_RTK_FIX=3
int32 GNSS_STATUS_RTK_FLT=4
int32 GNSS_STATUS_DR=5
int32 GNSS_STATUS_GNSSDR=6
int32 GNSS_STATUS_TIME=7
int32 GNSS_STATUS_SIM=8
int32 GNSS_STATUS_PPS_FIX=9

int32 status

# Many of these values would be better off defaulting to NaN, but that
# is currently not implemented in ROS 2
# seconds
float64 uncertainty_time
# degrees
float64 latitude
# meters
float64 uncertainty_latitutde
# degrees
float64 longitude
# meters
float64 uncertainty_longitude
# Altitude, height above ellipsoid (HAE)
float64 altitude_ellipsoid
# Altitude, height above mean sea level (MSL = HAE - geoid_separation)
float64 altitude_msl
# meters
float64 uncertainity_vertical_position
# Course made good (CMG), relative to true north
float64 track
# degrees
float64 uncertainty_track
# meters/second
float64 speed
float64 uncertainty_speed
float64 climb
# meters / second
float64 uncertainty_climb
# 2D horizontal error in meters. Exact definition is manufacturer dependent
float64 uncertainity_horizontal_position
# 3D sphereical error in meters. Exact definition is manufacturer dependent
float64 uncertainty_position
# meters. Positive is MSL above WGS84
float64 geoid_separation
# Course made good (CMG) relative to magnetic north
float64 track_magnetic
# Degrees
float64 magnetic_variation

gps_msgs/GNSSEcef ecef
gps_msgs/GNSSNed ned

char[40] datum

float64 dpgs_age -1