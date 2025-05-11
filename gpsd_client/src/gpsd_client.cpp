#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "gpsd_client_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpsd_client::GpsdClientNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}