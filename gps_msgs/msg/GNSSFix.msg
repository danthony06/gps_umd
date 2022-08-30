# A more complete GNSS fix description to supplement sensor_msgs/NavSatFix.
# The information in this message is derived from data in
# GPSD's gps_fix_t struct, but the mapping may not be one-to-one
# because that struct contains additional, non-GNSS related data,
# and is not a stable interface. Unless otherwise noted, invalid or
# not provided values will be NaNs
std_msgs/Header header

GNSSStatus status

# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude

# Longitude (degrees). Positive is east of prime meridian, negative west.
float64 longitude

# Altitude (meters). Positive is above ellipsoid (i.e. WGS84 or MSL)
float64 altitude

# Course made good relative to true north
float64 track

# Speed over ground (meters/second)
float64 speed

# Vertical speed (meters/second)
float64 climb

# GPS time
float64 time

## Dilution of precision values

# Total (positional-temporal) dilution of precision
float64 gdop

# Positional (3D) dilution of precision
float64 pdop

# Horizontal dilution of precision
float64 hdop

# Vertical dilution of precision
float64 vdop

# Temporal dilution of precision
float64 tdop

## Uncertainty of measurement. Nominally 95% confidence, but
# may be reported differently by different manufacturers

# Spherical position uncertainty (meters) [epe]
float64 err_position

# Horizontal position uncertainty (meters) [eph]
float64 err_horz

# Vertical position uncertainty (meters) [epv]
float64 err_vert

# Track uncertainty (degrees) [epd]
float64 err_track

# Ground speed uncertainty (meters/second) [eps]
float64 err_speed

# Vertical speed uncertainty (meters/second) [epc]
float64 err_climb

# Temporal uncertainty [ept]
float64 err_time

# Needs to be one of the constants defined below
int8 fix_status

int8 FIX_UNKNOWN = 0
int8 FIX_NORMAL = 1
int8 FIX_DGPS = 2
int8 FIX_RTK_FIXED = 3
int8 FIX_RTK_FLOATING = 4
int8 FIX_DEAD_RECKONING = 5
int8 FIX_GNSS_DEAD_RECKONING = 6
int8 FIX_TIME_SURVEYED = 7
int8 FIX_SIMULATED = 8
int8 FIX_P_Y_CODE = 9

# Datum the values are referenced in. Probably WGS-84, but early
# versions of GPSD did not report this. As of GPSD v14, this is limited
# to 40 characters, but we give a few extra for future proofing
string[<=64] datum

string DEFAULT_DATUM = "NOT REPORTED"
