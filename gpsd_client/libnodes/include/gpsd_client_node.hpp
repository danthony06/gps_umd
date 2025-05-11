#ifndef GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_
#define GPSD_CLIENT_GPSD_CLIENT_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_msgs/msg/gnss.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "gpsd_reader.hpp"
#include "types.hpp"

namespace gpsd_client
{
class GpsdClientNode : public rclcpp::Node
{
public:
  explicit GpsdClientNode(const rclcpp::NodeOptions& options);
  ~GpsdClientNode();

private:
  void timer_callback();
  void publish_gps_fix();
  void publish_nav_sat_fix();
  void publish_gnss_fix();

  bool use_gps_time_;
  bool check_fix_by_variance_;
  std::string frame_id_;
  int32_t publish_rate_;
  std::string host_;
  std::chrono::milliseconds publisher_period_ms_ {};
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t port_;

  std::unique_ptr<gnss_data> data_;
  std::unique_ptr<GpsdReader> gps_reader_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<gps_msgs::msg::GNSS>::SharedPtr gnss_fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

};
}

#endif
