#include "gpsd_parser.hpp"
#include "gpsd_parser_v11.hpp"

namespace gpsd_client
{
GpsdParser::GpsdParser()
{
  pimpl_ = std::make_unique<GpsdParserV11>();
}

GpsdParser::~GpsdParser() = default;

gps_msgs::msg::GNSS GpsdParser::parse(const gps_data_t& data)
{
  return pimpl_->parse(data);

}



}