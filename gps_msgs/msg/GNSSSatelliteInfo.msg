# Signal to noise ratio. Manufacturer defined ranges
float64 snr -1
# Whether satellite was used in fix
bool used false
# PRN of satellite. Manufacturer defined ranges
uint16 prn 0

# Satellite location
float64 elevation
float64 azimuth

# GNSS type. GPS, SBAS< Galileo, etc. Values are manufacturer specific
uint8 gnss_id 255
# Satellite vehicle ID. Manufacturer defined values
uint8 vehicle_id 255
# Signal ID. Manufacturer defined value
uint8 signal_id 255
# Frequency ID
int8 sig_id -128
# Vehicle health
uint8 VEHICLE_HEALTH_UNKNOWN=0
uint8 VEHICLE_HEALTH_HEALTHY=1
uint8 VEHICLE_HEALTH_UNHEALTHY=2
uint8 vehicle_health 0
