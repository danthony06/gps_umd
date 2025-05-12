#ifndef GPSD_CLIENT_GPSD_READER_IMPL_HPP_
#define GPSD_CLIENT_GPSD_READER_IMPL_HPP_

#include <memory>
#include <libgpsmm.h>
#include "gpsd_reader_impl_base.hpp"

namespace gpsd_client
{
class GpsdReaderImplV11 : public GpsdReaderImplBase
{
public:
  explicit GpsdReaderImplV11(const std::string& host, const std::string& port);
  ~GpsdReaderImplV11() override;
  std::unique_ptr<gnss_data> stream() override;
  bool waiting(const int t) override;
  std::unique_ptr<gnss_data> read() override;

private:
  std::unique_ptr<gpsmm> gps_;
};
}

#endif