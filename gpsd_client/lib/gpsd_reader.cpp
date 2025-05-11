#include "gpsd_reader.hpp"
#include "gpsd_reader_impl.hpp"

namespace gpsd_client
{
GpsdReader::GpsdReader() :
  pimpl_(std::make_unique<GpsdReaderImpl>("localhost", "123"))
{

}

GpsdReader::~GpsdReader() {}


gps_data_t GpsdReader::stream(const int flags)
{
  gps_data_t data;

  return data;
}

bool GpsdReader::waiting(const int t)
{
  return false;
}

gps_data_t GpsdReader::read()
{
  gps_data_t data;

  return data;
}

}