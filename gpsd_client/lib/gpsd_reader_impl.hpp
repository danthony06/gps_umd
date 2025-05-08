#ifndef GPSD_CLIENT_GPSD_READER_IMPL_HPP_
#define GPSD_CLIENT_GPSD_READER_IMPL_HPP_

#include <memory>
#include <libgpsmm.h>
#include "gpsd_reader_impl_base.hpp"

namespace gpsd_client
{
class GpsdReaderImpl : public GpsdReaderImplBase
{
public:
  explicit GpsdReaderImpl();
  ~GpsdReaderImpl() override;
  gps_data_t stream(const int flags) override;
  bool waiting(const int t) override;
  gps_data_t read() override;

private:
  std::unique_ptr<gpsmm> gps_;
};
}

#endif