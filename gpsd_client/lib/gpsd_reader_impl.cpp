#include "gpsd_reader_impl.hpp"

namespace gpsd_client
{

GpsdReaderImpl::GpsdReaderImpl(const std::string& host, const std::string& port) :
  GpsdReaderImplBase(host, port)
{

}

GpsdReaderImpl::~GpsdReaderImpl() {}

gps_data_t GpsdReaderImpl::stream(const int flags)
{
  gps_data_t data;

  return data;
}

bool GpsdReaderImpl::waiting(const int t)
{
  return false;
}

gps_data_t GpsdReaderImpl::read()
{
  gps_data_t data;

  return data;
}
}
