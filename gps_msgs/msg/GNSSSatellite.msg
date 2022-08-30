int16 prn # PRN identifier
float64 snr # SNR in dB, -1 for invalid
float64 elevation # Elevation
float64 azimuth # Azimuth
int8 gnss_id # GNSS ID, see definitions below
uint8 sv_id # Satellite vehicle ID
int8 freq_id # Frequency ID
int8 health

uint8 GNSS_INVALID_SV_ID = 0

# Different types of GNSS systems we could be using
int8 GNSS_ID_INVALID=-1 # Invalid
int8 GNSS_ID_GPS=0
int8 GNSS_ID_SBAS=1 # Satellite based augmentation system
int8 GNSS_ID_GALILEO=2
int8 GNSS_ID_BEIDOU=3
int8 GNSS_ID_IMES=4 # Indoor messaging system
int8 GNSS_ID_QZSS=5 # Quasi-zenith satellite system
int8 GNSS_ID_GLONASS=6 # GLONASS
int8 GNSS_ID_IRNSS=7 # India regional navigation satellite system

# Signal ID, as defined by U-blox 9
int8 SIG_ID_INVALID=-1
int8 SIG_ID_GPS_L1CA=0 # L1C/A
int8 SIG_ID_GPS_L2_CL=1 # L2 CL
int8 SIG_ID_GPS_L2_CM=2 # L2 CM
int8 SIG_ID_SBAS_L1CA=3 # L1C/A
int8 SIG_ID_SBAS_L5A=4 # L5A
int8 SIG_ID_GALILEO_E1_C=5 # E1 C
int8 SIG_ID_GALILEO_E1_B=6 # E1 B
int8 SIG_ID_GALILEO_E5_BL=7 # E5 bl
int8 SIG_ID_GALILEO_E5_BQ=8 # E5 bq
int8 SIG_ID_BEIDOU_B1I_D1=9 # B1I D1
int8 SIG_ID_BEIDOU_B1I_D2=10 # B1I D2
int8 SIG_ID_BEIDOU_B2I_D1=11 # B2I D1
int8 SIG_ID_BEIDOU_B2I_D2=12 # B2I D2
int8 SIG_ID_QZSS_L1CA=13 # L1C/A
int8 SIG_ID_QZSS_L2_CM=14 # L2 CM
int8 SIG_ID_QZSS_L2_CL=15 # L2 CL
int8 SIG_ID_GLONASS_L1_OF=16 # L1 OF
int8 SIG_ID_GLONASS_L2_OF=17 # L2 OF

# Satellite health
int8 GNSS_SAT_HEALTH_UNKNOWN=-1
int8 GNSS_SAT_HEALTH_OK=0
int8 GNSS_SAT_HEALTH_BAD=1
