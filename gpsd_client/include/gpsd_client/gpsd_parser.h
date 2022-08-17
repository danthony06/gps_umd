#ifndef GPSD_CLIENT_GPSD_PARSER_H_
#define GPSD_CLIENT_GPSD_PARSER_H_

#include <string>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace gpsd_client
{

class GpsdParser
{
  public:
    GpsdParser(const std::string& host, const int port) {}
    virtual bool getData(
      sensor_msgs::msg::NavSatFix& fix_msg,
      sensor_msgs::msg::NavSatStatus& status_msg) = 0;
};

}
#endif // GPSD_CLIENT_GPSD_PARSER_H_