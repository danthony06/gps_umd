#ifndef GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_
#define GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_

#include <libgpsmm.h>

#include <rclcpp/rclcpp.hpp>

#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_msgs/msg/gnss.hpp>
//#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace gpsd_client
{
class GpsdClientNode : public rclcpp::Node
{
public:
  explicit GpsdClientNode(const rclcpp::NodeOptions& options);
  ~GpsdClientNode();

private:
  bool use_gps_time_;
  bool check_fix_by_variance_;
  std::string frame_id_;
  int32_t publish_rate_;
  std::string host_;
  std::chrono::milliseconds publisher_period_ms_ {};
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t port_;
  
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<gps_msgs::msg::GNSS>::SharedPtr gnss_fix_pub_;
  //rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  std::unique_ptr<gpsmm> gps_;
};
}

#endif
