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
      gps_msgs::msg::GNSSFix& fix_msg,
      gps_msgs::msg::GNSSStatus& status_msg) = 0;

  private:
    std::unique_ptr<gpsmm> gps_;
    int8_t map_freq(const int8_t gnss_id, const int8_t freq_id);
};
}