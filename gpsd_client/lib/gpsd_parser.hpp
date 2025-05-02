#ifndef GPSD_CLIENT_GPSD_PARSER_HPP_
#define GPSD_CLIENT_GPSD_PARSER_HPP_

#include <memory>
#include <gps.h>

#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{

class AbstractGpsdParserImpl;

class GpsdParser
{
public:
  explicit GpsdParser();
  ~GpsdParser();

  gps_msgs::msg::GNSS parse(const gps_data_t& data);

private:
  std::unique_ptr<AbstractGpsdParserImpl> pimpl_;

};

}

#endif