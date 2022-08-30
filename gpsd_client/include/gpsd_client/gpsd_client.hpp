#ifndef GPSD_CLIENT_GPSD_CLIENT_HPP_
#define GPSD_CLIENT_GPSD_CLIENT_HPP_

#include <libgpsmm.h>

#include <rclcpp/rclcpp.hpp>

#include <gps_msgs/msg/gnss_fix.hpp>

namespace gpsd_client
{
class GPSDClientComponent : public rclcpp::Node
{
  public:
    explicit GPSDClientComponent(const rclcpp::NodeOptions& options);

  private:
    void step();

    rclcpp::Publisher<gps_msgs::msg::GNSSFix>::SharedPtr gps_fix_pub_;

    std::unique_ptr<gpsmm> gps_;

    bool use_gps_time_;
    bool check_fix_by_variance_;
    std::string frame_id_;
    int publish_rate_;
    std::chrono::milliseconds publish_period_ms{};
    rclcpp::TimerBase::SharedPtr timer_;
};
}


#endif // GPSD_CLIENT_GPSD_CLIENT_HPP_