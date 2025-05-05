#ifndef GPSD_PARSER_GPSD_PARSER_V11_HPP_
#define GPSD_PARSER_GPSD_PARSER_V11_HPP_

#include <gps.h>
#include <gps_msgs/msg/gnss.hpp>
namespace gpsd_client
{
gps_msgs::msg::GNSS parse_v11(const gps_data_t& data)
{
  gps_msgs::msg::GNSS msg;

  return msg;
}
}

#endif