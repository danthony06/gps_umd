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
    GpsdParser(const std::string& host, const int port);
    ~GpsdParser();
    bool getData(
      gps_msgs::msg::GNSSFix& fix_msg,
      gps_msgs::msg::GNSSStatus& status_msg);
    
    private:
      class GpsdParserImpl; std::unique_ptr<GpsdParserImpl> pimpl_;

      // This is a workaround to current limitations in the rosidl message
      // initialization
      void initializeMessages(
        gps_msgs::msg::GNSSFix& fix_msg,
        gps_msgs::msg::GNSSStatus& status_msg);

};

}
#endif // GPSD_CLIENT_GPSD_PARSER_H_