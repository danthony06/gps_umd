#include <rcpputils/asserts.hpp>
#include <gpsd_client/gpsd_parser_v9.h>
#include <libgpsmm.h>
#include <map>

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
      gps_msgs::msg::GNSSFix& fix_msg,
      gps_msgs::msg::GNSSStatus& status_msg)
{
  if (gps_ == nullptr || !gps_->is_open()) { return false; }

  // The read() call returns a raw pointer to an internal class member. The access
  // is not thread safe, and the class maintains ownership over the data, so do not
  // free the memory in this function.
  if (!gps_->waiting(1e6)) {return false; }
  gps_data_t* p = gps_->read();
  if (p == nullptr) { return false; }
  if (!(p->online.tv_sec || p->online.tv_nsec)) { return false; }

  fix_msg.header.stamp.sec = p->fix.time.tv_sec;
  fix_msg.header.stamp.nanosec = p->fix.time.tv_nsec;
  fix_msg.altitude = p->fix.altitude;
  status_msg.num_satellites_used = p->satellites_used;
  status_msg.num_satellites_unused = std::max(
    0,
    p->satellites_visible - p->satellites_used);
  status_msg.satellites_used.resize(
    std::max(
      static_cast<size_t>(0),
      std::min(
        static_cast<size_t>(p->satellites_used),
        status_msg.satellites_used.max_size())));

  rcpputils::assert_true(
    status_msg.satellites_unused.size() + status_msg.satellites_used.size() == 
    p->satellites_visible);

  for (int visible_idx, used_idx, unused_idx = 0; visible_idx < p->satellites_visible; visible_idx++)
  {
    gps_msgs::msg::GNSSSatellite::RawPtr current_sat = nullptr;
    if (p->skyview[visible_idx].used)
    {
      current_sat = &status_msg.satellites_used[used_idx];
      used_idx++;
    }
    else
    {
      current_sat = &status_msg.satellites_unused[unused_idx];
      unused_idx++;
    }

    current_sat->prn = p->skyview[visible_idx].PRN;
    current_sat->snr= p->skyview[visible_idx].ss;
    current_sat->elevation = p->skyview[visible_idx].elevation;
    current_sat->azimuth = p->skyview[visible_idx].azimuth;

    if (p->skyview[visible_idx].svid != 0)
    {
      current_sat->gnss_id = p->skyview[visible_idx].gnssid;
      current_sat->sv_id = p->skyview[visible_idx].svid;
      current_sat->freq_id = p->skyview[visible_idx].freqid;
    }
    else
    {
      current_sat->sv_id = gps_msgs::msg::GNSSSatellite::GNSS_INVALID_SV_ID;
      current_sat->gnss_id = gps_msgs::msg::GNSSSatellite::GNSS_ID_INVALID;
      current_sat->freq_id = gps_msgs::msg::GNSSSatellite::SIG_ID_INVALID;
    }
    
  }
  return true;
}

int8_t GpsdParserV9::map_freq(const int8_t gnss_id, const int8_t freq_id)
{
  const std::map<int8_t, std::map<int8_t, int8_t>> freq_map =
  {
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_GPS, {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L1CA},
      {3, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L2_CL},
      {4, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L2_CM}}}
  };

  return 0;
}
}
