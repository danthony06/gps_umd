#ifndef GPSD_CLIENT_TYPES_HPP_
#define GPSD_CLIENT_TYPES_HPP_

#include <cstdint>
#include <sys/time.h>
#include <vector>

namespace gpsd_client
{
  struct gnss_data
  {
    timespec fix_time;
    std::vector<uint16_t> used_satellite_prns;
    std::vector<uint16_t> visible_satellite_prns;
    std::vector<int32_t> visible_satellite_elevation;
    std::vector<int32_t> visible_satellite_azimuth;
    std::vector<int32_t> visible_satellite_snr;
    int8_t fix_mode;
    int8_t fix_type;
    double uncertainty_time;
    double latitude;    
    double epy;         
    double longitude;   
    double epx;         
    double altitude;    
    double altitude_above_ellipsoid;      
    double altitude_msl;
    double uncertainty_vertical;
    double track;
    double uncertainty_track;
    double sog;
    double uncertainty_speed;
    double climb;
    double uncertainty_vertical;
    double uncertainty_2d;
    double uncertainty_3d;
    double geoid_separation;
    double course_magnetic;
    double magnetic_variation;
    struct {
        double x;
        double y;
        double z;
        double uncertainty_3d;
        double vx;
        double vy;
        double vz;      // ECEF x, y, z velocity
        double vAcc;            // Velocity Accuracy Estimate, probably SEP
    } ecef;
    // NED data, all data in meters, and meters/second, or NaN
    struct {
        double relPosN, relPosE, relPosD;   // NED relative positions
        double relPosL, relPosH;            // relative length and heading
        double velN, velE, velD;            // NED velocities
    } NED;
    char datum[40];             // map datum
    // DGPS stuff, often from xxGGA, or xxGNS
    double dgps_age;       // age of DGPS data in seconds, -1 invalid
    /* DGPS Station used, max size is a guess
     * NMEA 2 says 0000-1023
     * RTCM 3, station ID is 0 to 4095.
     * u-blox UBX-NAV-DGPS is 16 bit integer */
    int dgps_station;           // DGPS station ID, -1 invalid
    double wanglem;             // Wind angle, magnetic, m/s
    double wangler;             // Wind angle, relative, m/s
    double wanglet;             // Wind angle, true, m/s
    double wspeedr;             // Wind speed, relative, m/s
    double wspeedt;             // Wind speed, true, m/s
    struct baseline_t base;     // baseline from fixed base
  };
}


#endif