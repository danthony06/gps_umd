#include "gpsd_parser.hpp"
//#include "gpsd_parser_V11.hpp"

namespace gpsd_client
{
class GpsdParser::AbstractGpsdParserImpl
{
public:
  AbstractGpsdParserImpl() {}
  gps_msgs::msg::GNSS parse(const gps_data_t& data) const
  {
   gps_msgs::msg::GNSS msg;
   return msg;
  }

};

GpsdParser::GpsdParser() :
  pimpl_(new AbstractGpsdParserImpl())
{

}

GpsdParser::GpsdParser(const GpsdParser& other) :
  pimpl_(new AbstractGpsdParserImpl())
{

}

GpsdParser::~GpsdParser() {}

GpsdParser& GpsdParser::operator=(const GpsdParser& other)
{
  pimpl_ = std::unique_ptr<AbstractGpsdParserImpl>(new AbstractGpsdParserImpl());
  return *this;
}

}
