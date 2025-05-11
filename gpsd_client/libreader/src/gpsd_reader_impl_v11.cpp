#include "gpsd_reader_impl_v11.hpp"

namespace gpsd_client
{

GpsdReaderImplV11::GpsdReaderImplV11(const std::string& host, const std::string& port) :
  GpsdReaderImplBase(host, port),
  gps_(new gpsmm(host.c_str(), port.c_str()))
{

}

GpsdReaderImplV11::~GpsdReaderImplV11() {}

std::unique_ptr<gps_data_t> GpsdReaderImplV11::stream(const int flags)
{
  return std::move(std::unique_ptr<gps_data_t>(gps_->stream(WATCH_ENABLE)));
}

bool GpsdReaderImplV11::waiting(const int t)
{
  return false;
}

std::unique_ptr<gps_data_t> GpsdReaderImplV11::read()
{
  std::unique_ptr<gps_data_t> data;

  return std::move(data);
}
}
