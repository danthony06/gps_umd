#include <gpsd_client/gpsd_client_node.hpp>

namespace gpsd_client
{
GpsdClientNode::GpsdClientNode(const rclcpp::NodeOptions& options) :
  Node("gpsd_client", options)
{

}

GpsdClientNode::~GpsdClientNode()
{

}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GpsdClientNode)