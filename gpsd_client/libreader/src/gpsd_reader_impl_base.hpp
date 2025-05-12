#ifndef GPSD_PARSER_GPSD_READER_IMPL_BASE_HPP_
#define GPSD_PARSER_GPSD_READER_IMPL_BASE_HPP_

#include <string>
#include "types.hpp"

namespace gpsd_client
{
class GpsdReaderImplBase
{
public:
  explicit GpsdReaderImplBase(const std::string& host, const std::string& port) {}
  virtual ~GpsdReaderImplBase() = default;
  virtual std::unique_ptr<gnss_data> stream() = 0;
  virtual bool waiting(const int t) = 0;
  virtual std::unique_ptr<gnss_data> read() = 0;
};
}

#endif