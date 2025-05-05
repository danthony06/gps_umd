#ifndef GPSD_CLIENT_GPSD_PARSER_FACTORY_HPP_
#define GPSD_CLIENT_GPSD_PARSER_FACTORY_HPP_

#include <functional>
#include <memory>
#include <gps.h>

#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{
class GpsdParser
{
public:
  explicit GpsdParser();

  std::function<gps_msgs::msg::GNSS(const gps_data_t&)> parse;
};

}

#endif