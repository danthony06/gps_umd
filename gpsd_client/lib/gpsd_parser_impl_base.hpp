#ifndef GPSD_PARSER_GPSD_PARSER_IMPL_BASE_HPP_
#define GPSD_PARSER_GPSD_PARSER_IMPL_BASE_HPP_

#include <gps.h>

#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{
class GpsdParserImplBase
{
public:
  virtual ~GpsdParserImplBase() = default;
  virtual void parse(const gps_data_t& data, gps_msgs::msg::GNSS& msg) = 0;
};

}

#endif