std_msgs/Header header

int8 mode

# Satellites used in solution
uint16 num_satellites_used # Number of satellites
int16[<=72] satellite_used_prn # PRN identifiers
float64[<=72] satellites_used_elevation # Elevation of each used satellite
float64[<=72] satellites_used_azimuth
float64[<=72] satellites_used_residual
int8[<=72] satellites_used_quality
uint8[<=72] satellites_used_id

# Satellites visible but not used
uint16 num_satellites_visible
int16[<=72] satellite_visible_prn # PRN identifiers
float64[<=72] satellites_visible_elevation # Elevation of each used satellite
float64[<=72] satellites_visible_azimuth
float64[<=72] satellites_visible_residual
int8[<=72] satellites_visible_quality
uint8[<=72] satellites_visible_id

# Consants related to mode
int8 MODE_UNKNOWN = -1
int8 MODE_NO_FIX = 0
int8 MODE_2D = 1
int8 MODE_3D = 2

# Constants related to quality
int8 QUALITY_INVALID = -1
int8 QUALITY_NO_SIGNAL = 0
int8 QUALITY_SEARCHING = 1
int8 QUALITY_ACQUIRED = 2
int8 QUALITY_DETECTED = 3
int8 QUALITY_CODE_LOCKED = 4
int8 QUALITY_CODE_CARRIER_LOCKED = 5
