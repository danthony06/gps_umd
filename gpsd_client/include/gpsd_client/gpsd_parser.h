#ifndef GPSD_CLIENT_GPSD_PARSER_H_
#define GPSD_CLIENT_GPSD_PARSER_H_

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
};

}
#endif // GPSD_CLIENT_GPSD_PARSER_H_