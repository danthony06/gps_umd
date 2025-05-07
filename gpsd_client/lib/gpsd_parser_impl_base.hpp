#ifndef GPSD_PARSER_GPSD_PARSER_IMPL_BASE_HPP_
#define GPSD_PARSER_GPSD_PARSER_IMPL_BASE_HPP_

#include "gps.h"

namespace gpsd_client
{
class GpsdParserImplBase
{
public:
  virtual ~GpsdParserImplBase() = default;
  virtual void parse(const gps_data_t& data) = 0;
};

}

#endif