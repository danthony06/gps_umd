#ifndef GPSD_PARSER_GPSD_PARSER_IMPL_V11_HPP_
#define GPSD_PARSER_GPSD_PARSER_IMPL_V11_HPP_

#include "gpsd_parser_impl_base.hpp"
#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{
class GpsdParserImplV11 : public GpsdParserImplBase
{
public:
  explicit GpsdParserImplV11();
  ~GpsdParserImplV11() override;

  void parse(const gps_data_t& data) override;
};
}

#endif