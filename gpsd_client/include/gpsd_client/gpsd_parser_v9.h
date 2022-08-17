#include <gpsd_client/gpsd_parser.h>
#include <memory>
#include <libgpsmm.h>

namespace gpsd_client
{
class GpsdParserV9 : public GpsdParser
{
  public:
    GpsdParserV9(const std::string& host, const int port);
    bool getData(
      sensor_msgs::msg::NavSatFix& fix_msg,
      sensor_msgs::msg::NavSatStatus& status_msg) = 0;

  private:
    std::unique_ptr<gpsmm> gps_;
};
}