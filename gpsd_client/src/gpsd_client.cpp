#include <memory>
#include <gpsd_client/gpsd_client_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpsd_client::GpsdClientNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}