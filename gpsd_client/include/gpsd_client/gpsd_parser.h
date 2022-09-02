#ifndef GPSD_CLIENT_GPSD_PARSER_H_
#define GPSD_CLIENT_GPSD_PARSER_H_

#include <cmath>
#include <string>

#include <gps_msgs/msg/gnss_fix.hpp>
#include <gps_msgs/msg/gnss_status.hpp>

namespace gpsd_client
{

class GpsdParser
{
  public:
    GpsdParser(const std::string& host, const int port) {}
    virtual bool getData(
      gps_msgs::msg::GNSSFix& fix_msg,
      gps_msgs::msg::GNSSStatus& status_msg) = 0;

    // This is a workaround to current limitations in the rosidl message
    // initialization
    void initialize_messages(
      gps_msgs::msg::GNSSFix& fix_msg,
      gps_msgs::msg::GNSSStatus& status_msg)
    {
      fix_msg.time.sec = 0;
      fix_msg.time.nanosec = 0;
      // Currently do not have a clean way of initializing message memebers to
      // NaN, so this makes it explicit
      fix_msg.latitude = NAN;
      fix_msg.longitude = NAN;
      fix_msg.altitude = NAN;
      fix_msg.track = NAN;
      fix_msg.speed = NAN;
      fix_msg.climb = NAN;
      fix_msg.gdop = NAN;
      fix_msg.hdop = NAN;
      fix_msg.pdop = NAN;
      fix_msg.vdop = NAN;
      fix_msg.tdop = NAN;
      fix_msg.err_position = NAN;
      fix_msg.err_horz = NAN;
      fix_msg.err_vert = NAN;
      fix_msg.err_track = NAN;
      fix_msg.err_speed = NAN;
      fix_msg.err_climb = NAN;
      fix_msg.err_time = NAN;
      fix_msg.err_latitude = NAN;
      fix_msg.err_longitude = NAN;
      // Also do not have a clean way of initializing a bounded string vector
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_UNKNOWN;
      std::string temp_datum = gps_msgs::msg::GNSSFix::DEFAULT_DATUM;
      fix_msg.datum.resize(std::min(fix_msg.datum.max_size(), temp_datum.length()));
      for (int copy_idx = 0; copy_idx < fix_msg.datum.size(); copy_idx++)
      {
        fix_msg.datum[copy_idx] = temp_datum.at(copy_idx);
      }

      // Cannot symbolically initialize member in the message definition, so do it here
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_UNKNOWN;
      // This is probably not necessary, but makes the initialization explicit
      status_msg.num_satellites_used = 0;
      status_msg.num_satellites_unused = 0;
      status_msg.satellites_used.clear();
      status_msg.satellites_unused.clear();

      fix_msg.status = status_msg;
    }
};

}
#endif // GPSD_CLIENT_GPSD_PARSER_H_