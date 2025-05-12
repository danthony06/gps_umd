#include <gps.h>
#include "gpsd_reader_impl_v11.hpp"

namespace gpsd_client
{

GpsdReaderImplV11::GpsdReaderImplV11(const std::string& host, const std::string& port) :
  GpsdReaderImplBase(host, port),
  gps_(new gpsmm(host.c_str(), port.c_str()))
{

}

GpsdReaderImplV11::~GpsdReaderImplV11() {}

std::unique_ptr<gnss_data> GpsdReaderImplV11::stream()
{
  auto gps_data = std::move(gps_->stream(WATCH_ENABLE));

  std::unique_ptr<gnss_data> data;

  return std::move(data);
}

bool GpsdReaderImplV11::waiting(const int t)
{
  return false;
}

std::unique_ptr<gnss_data> GpsdReaderImplV11::read()
{
  std::unique_ptr<gnss_data> data;

  return std::move(data);
}
}
