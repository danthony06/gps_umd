#include <memory>
#include <utility>

#include "gpsd_parser.hpp"
#include "gpsd_parser_impl_v11.hpp"

namespace gpsd_client
{
GpsdParser::GpsdParser()
{
  pimpl_ = std::make_unique<GpsdParserImplV11>();
}

GpsdParser::~GpsdParser() {}

void GpsdParser::parse(const gps_data_t& data)
{
  pimpl_->parse(data);
}
}
