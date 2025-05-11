#include "gpsd_client_node.hpp"

namespace gpsd_client
{
GpsdClientNode::GpsdClientNode(const rclcpp::NodeOptions& options) :
  Node("gpsd_client", options)
{
  this->declare_parameter("use_gps_time", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("check_fix_by_variance", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  this->declare_parameter("publish_rate", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("host", rclcpp::PARAMETER_STRING);
  this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);

  gps_fix_pub_ = create_publisher<gps_msgs::msg::GPSFix>("extended_fix", 1);
  nav_sat_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
  gnss_fix_pub_ = create_publisher<gps_msgs::msg::GNSS>("gnss", 1);

  std::string host = "localhost";
  int port = 2947;
  this->get_parameter_or("host", host, host);
  this->get_parameter_or("port", port, port);
  this->get_parameter_or("use_gps_time", use_gps_time_, use_gps_time_);
  this->get_parameter_or("check_fix_by_variance", check_fix_by_variance_, check_fix_by_variance_);
  this->get_parameter_or("frame_id", frame_id_, frame_id_);
  this->get_parameter_or("publish_rate", publish_rate_, publish_rate_);

  publisher_period_ms_ = std::chrono::milliseconds{(int)(1000 / publish_rate_)};

  auto resp = gps_reader_->stream();
  if (resp == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open GPS device");
  }
  else
  {
    timer_ = create_wall_timer(
      publisher_period_ms_,
      std::bind(&GpsdClientNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Connected to GPS device.");
  }
}

GpsdClientNode::~GpsdClientNode()
{

}

void GpsdClientNode::timer_callback()
{
  while (gps_reader_->waiting(0))
  {
    data_ = std::move(gps_reader_->read());
  }

  if ((data_ == nullptr) ||
    (!data_->online.tv_sec && !data_->online.tv_nsec))
  {
    return;
  }

  publish_gps_fix();
  publish_nav_sat_fix();
  publish_gnss_fix();
}

void GpsdClientNode::publish_gps_fix()
{

}

void GpsdClientNode::publish_nav_sat_fix()
{

}

void GpsdClientNode::publish_gnss_fix()
{

}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GpsdClientNode)