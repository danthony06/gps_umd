#ifndef GPSD_CLIENT_GPSD_PARSER_HPP_
#define GPSD_CLIENT_GPSD_PARSER_HPP_

#include <memory>
#include <gps.h>

#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{
class GpsdParser
{
public:
  explicit GpsdParser();
  GpsdParser(const GpsdParser& other); // Copy
  GpsdParser(GpsdParser&& other); // Move
  virtual ~GpsdParser();
  GpsdParser& operator=(const GpsdParser& other);
  GpsdParser& operator=(GpsdParser&& other) = default; // Move assign

  gps_msgs::msg::GNSS parse(const gps_data_t& data) const;

private:
  class AbstractGpsdParserImpl;
  std::unique_ptr<AbstractGpsdParserImpl> pimpl_;

};

}

#endif