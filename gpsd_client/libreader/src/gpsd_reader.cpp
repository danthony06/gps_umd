#include "gpsd_reader.hpp"
#include "gpsd_reader_impl_v11.hpp"

namespace gpsd_client
{
GpsdReader::GpsdReader(const std::string& host, const std::string& port) :
  pimpl_(std::make_unique<GpsdReaderImplV11>(host, port))
{
}

GpsdReader::GpsdReader(GpsdReader&& other) noexcept :
  pimpl_(std::move(other.pimpl_))
{

}

GpsdReader::~GpsdReader() {}

std::unique_ptr<gnss_data> GpsdReader::stream()
{
  std::unique_ptr<gnss_data> data_;
  auto raw_data = std::move(pimpl_->stream(WATCH_ENABLE));
  return std::move(data_);
}

bool GpsdReader::waiting(const int t)
{
  return pimpl_->waiting(t);
}

std::unique_ptr<gnss_data> GpsdReader::read()
{
  std::unique_ptr<gnss_data> data;
  auto raw_data = std::move(pimpl_->read());
  return std::move(data);
}

}