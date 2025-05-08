#ifndef GPSD_CLIENT_GPSD_READER_HPP_
#define GPSD_CLIENT_GPSD_READER_HPP_

#include <functional>
#include <memory>
#include <gps.h>

namespace gpsd_client
{
class GpsdReaderImplBase;

class GpsdReader
{
public:
  explicit GpsdReader();
  ~GpsdReader();
  GpsdReader(const GpsdReader&) = delete;
  GpsdReader(GpsdReader&& other) noexcept;
  GpsdReader& operator=(GpsdReader&& other) noexcept;

  gps_data_t stream(const int flags);
  bool waiting(const int t);
  gps_data_t read();

private:
  std::unique_ptr<GpsdReaderImplBase> pimpl_;
};
}

#endif
