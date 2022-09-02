#include <libgpsmm.h>

#include <rclcpp/rclcpp.hpp>
#include <gpsd_client/gpsd_client.hpp>
#include <gps_msgs/msg/gnss_status.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace gpsd_client
{
GPSDClientComponent::GPSDClientComponent(const rclcpp::NodeOptions& options) :
  Node("gpsd_client", options),
  gps_(nullptr),
  use_gps_time_(true),
  check_fix_by_variance_(true),
  frame_id_("gps"),
  publish_rate_(1)
{
  use_gps_time_ = this->declare_parameter<bool>("use_gps_time", use_gps_time_);
  check_fix_by_variance_ = this->declare_parameter<bool>("check_fix_by_variance", check_fix_by_variance_);
  frame_id_ = this->declare_parameter<std::string>("frame_id", frame_id_);
  publish_rate_ = this->declare_parameter<int>("publish_rate", publish_rate_);

  publish_period_ms = std::chrono::milliseconds{(int)(1000 / publish_rate_)};

  std::string host = "localhost";
  int port = 2947;
  this->get_parameter_or("host", host, host);
  this->get_parameter_or("port", port, port);

  gps_fix_pub_ = create_publisher<gps_msgs::msg::GNSSFix>("extended_fix", 1);
  // TODO: Open stream here 

  timer_ = create_wall_timer(publish_period_ms, std::bind(&GPSDClientComponent::step, this));
  RCLCPP_INFO(this->get_logger(), "Instantiated.");
}

void GPSDClientComponent::step()
{
}

} // namespace gpsd_client

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GPSDClientComponent)
