#ifndef GPSD_CLIENT_GPSD_PARSER_IMPL_HPP_
#define GPSD_CLIENT_GPSD_PARSER_IMPL_HPP_

#include <gps.h>
#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{
class AbstractGpsdParserImpl
{
public:
  virtual ~AbstractGpsdParserImpl() = default;
  virtual gps_msgs::msg::GNSS parse(const gps_data_t& data) = 0;
};

}

#endif