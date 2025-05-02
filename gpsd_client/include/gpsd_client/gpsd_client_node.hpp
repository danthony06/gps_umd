#ifndef GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_
#define GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace gpsd_client
{
class GpsdClientNode : public rclcpp::Node
{
public:
  explicit GpsdClientNode(const rclcpp::NodeOptions& options);
  ~GpsdClientNode();

};
}

#endif
