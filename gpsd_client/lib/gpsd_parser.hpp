#ifndef GPSD_CLIENT_GPSD_PARSER_HPP_
#define GPSD_CLIENT_GPSD_PARSER_HPP_

#include <functional>
#include <memory>
#include <gps.h>

#include <gps_msgs/msg/gnss.hpp>

namespace gpsd_client
{

class GpsdParserImplBase;

class GpsdParser
{
public:
  explicit GpsdParser();
  ~GpsdParser();
  GpsdParser(const GpsdParser&) = delete;
  GpsdParser(GpsdParser&& other) noexcept;
  GpsdParser& operator=(GpsdParser&& other) noexcept;

  gps_msgs::msg::GNSS parse(const gps_data_t& data);

private:
  std::unique_ptr<GpsdParserImplBase> pimpl_;
  gps_msgs::msg::GNSS make_message();
};

}

#endif
