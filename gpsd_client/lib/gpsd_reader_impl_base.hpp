#ifndef GPSD_PARSER_GPSD_READER_IMPL_BASE_HPP_
#define GPSD_PARSER_GPSD_READER_IMPL_BASE_HPP_

#include <string>

#include <gps.h>

namespace gpsd_client
{
class GpsdReaderImplBase
{
public:
  explicit GpsdReaderImplBase(const std::string& host, const std::string& port)
  {

  }
  virtual ~GpsdReaderImplBase() = default;
  virtual gps_data_t stream(const int flags) = 0;
  virtual bool waiting(const int t) = 0;
  virtual gps_data_t read() = 0;
};
}

#endif