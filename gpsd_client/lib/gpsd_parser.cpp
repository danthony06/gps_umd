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

  msg.fix.uncertainty_time = std::numeric_limits<double>::quiet_NaN();
  msg.fix.latitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_latitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.longitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_longitude = std::numeric_limits<double>::quiet_NaN();
  msg.fix.altitude_ellipsoid = std::numeric_limits<double>::quiet_NaN();
  msg.fix.altitude_msl = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainity_vertical_position = std::numeric_limits<double>::quiet_NaN();
  msg.fix.track = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_track = std::numeric_limits<double>::quiet_NaN();
  msg.fix.speed = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_speed = std::numeric_limits<double>::quiet_NaN();
  msg.fix.climb = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_climb = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_horizontal_position = std::numeric_limits<double>::quiet_NaN();
  msg.fix.uncertainty_position = std::numeric_limits<double>::quiet_NaN();
  msg.fix.geoid_separation = std::numeric_limits<double>::quiet_NaN();
  msg.fix.track_magnetic = std::numeric_limits<double>::quiet_NaN();
  msg.fix.magnetic_variation = std::numeric_limits<double>::quiet_NaN();

  msg.fix.ecef.x = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.y = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.z = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.position_accuracy = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_x = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_y = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_z = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_uncertainty_x = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_uncertainty_y = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ecef.velocity_uncertainty_z = std::numeric_limits<double>::quiet_NaN();

  msg.fix.ned.rel_position_north = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_position_east = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_position_down = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_position_length = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_position_heading = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_velocity_north = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_velocity_east = std::numeric_limits<double>::quiet_NaN();
  msg.fix.ned.rel_velocity_down = std::numeric_limits<double>::quiet_NaN();

  return msg;
}
}
