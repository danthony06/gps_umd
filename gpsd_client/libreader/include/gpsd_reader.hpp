#ifndef GPSD_CLIENT_GPSD_READER_HPP_
#define GPSD_CLIENT_GPSD_READER_HPP_

#include <functional>
#include <memory>
#include "types.hpp"

namespace gpsd_client
{
class GpsdReaderImplBase;

class GpsdReader
{
public:
  explicit GpsdReader(const std::string& host, const std::string& port);
  ~GpsdReader();
  GpsdReader(const GpsdReader&) = delete;
  GpsdReader(GpsdReader&& other) noexcept;
  GpsdReader& operator=(GpsdReader&& other) = delete;

  std::unique_ptr<gnss_data> stream();
  bool waiting(const int t);
  std::unique_ptr<gnss_data> read();

private:
  std::unique_ptr<GpsdReaderImplBase> pimpl_;
};
}

#endif
