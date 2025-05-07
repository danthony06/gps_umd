#include <limits>
#include <memory>
#include <utility>

#include <gps.h>

#include "gpsd_parser.hpp"

#if GPSD_API_MAJOR_VERSION == 11
#include "gpsd_parser_impl_v11.hpp"
#endif

namespace gpsd_client
{
GpsdParser::GpsdParser() :
  pimpl_(std::make_unique<GpsdParserImpl>())
{
}

GpsdParser::~GpsdParser() {}

gps_msgs::msg::GNSS GpsdParser::parse(const gps_data_t& data)
{
  auto msg = make_message();
  if (!pimpl_) { return msg; }

  pimpl_->parse(data, msg);

  return msg;
}

gps_msgs::msg::GNSS GpsdParser::make_message()
{
  // Currently in some ROS 2 distros it is not possible to initialize
  // message fields to NaN by default, so this function is necessary
  // to correctly initialize a new message
  gps_msgs::msg::GNSS msg;

  msg.fix.latitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.longitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.altitude_ellipsoid = std::numeric_limits<double>::quiet_NaN();
  msg.fix.altitude_msl = std::numeric_limits<double>::quiet_NaN();

  return msg;
}
}
