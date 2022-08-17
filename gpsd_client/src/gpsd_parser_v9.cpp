#include <gpsd_client/gpsd_parser_v9.h>
#include <libgpsmm.h>

namespace gpsd_client
{

GpsdParserV9::GpsdParserV9(const std::string& host, const int port) : 
  GpsdParser(host, port)
{
  std::string port_s = std::to_string(port);

  gps_ = std::unique_ptr<gpsmm>(new gpsmm(host.c_str(), port_s.c_str()));
  if (gps_->stream(WATCH_ENABLE) == nullptr)
  {
    // Handle error
  }
}

bool GpsdParserV9::getData(
      sensor_msgs::msg::NavSatFix& fix_msg,
      sensor_msgs::msg::NavSatStatus& status_msg)
{
  if (gps_ == nullptr || !gps_->is_open()) { return false; }

  // The read() call returns a raw pointer to an internal class member. The access
  // is not thread safe, and the class maintains ownership over the data, so do not
  // free the memory in this function.
  gps_data_t* p = gps_->read();
  if (p == nullptr) { return false; }

  return true;
}

}
