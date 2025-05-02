#ifndef GPSD_PARSER_GPSD_PARSER_V11_HPP_
#define GPSD_PARSER_GPSD_PARSER_V11_HPP_

#include "abstract_gpsd_parser_impl.hpp"

namespace gpsd_client
{
class GpsdParserV11 : AbstractGpsdParserImpl
{
public:
  gps_msgs::msg::GNSS parser(const gps_data_t& data)
  {
    gps_msgs::msg::GNSS msg;

    return msg;
  }

};
}

#endif