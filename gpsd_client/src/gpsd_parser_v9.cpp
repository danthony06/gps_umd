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
  fix_msg.err_time = p->fix.ept;
  fix_msg.altitude = p->fix.altitude;
  fix_msg.err_vert = p->fix.epv;
  fix_msg.latitude = p->fix.latitude;
  fix_msg.err_latitude = p->fix.epy;
  fix_msg.longitude = p->fix.longitude;
  fix_msg.err_longitude = p->fix.epx;
  fix_msg.err_position = p->fix.eph;
  fix_msg.track = p->fix.track;
  fix_msg.err_track = p->fix.epd;
  fix_msg.track_magnetic = p->fix.magnetic_track;
  fix_msg.magnetic_var = p->fix.magnetic_var;
  fix_msg.speed = p->fix.speed;
  fix_msg.err_speed = p->fix.eps;
  fix_msg.climb = p->fix.climb;
  fix_msg.err_climb = p->fix.epc;
  fix_msg.gdop = p->dop.gdop;
  fix_msg.hdop = p->dop.hdop;
  fix_msg.vdop = p->dop.vdop;
  fix_msg.tdop = p->dop.tdop;

  switch (p->fix.mode)
  {
    case MODE_NOT_SEEN:
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_NOT_SEEN;
      break;

    case MODE_NO_FIX:
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_NO_FIX;
      break;

    case MODE_2D:
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_2D;
      break;

    case MODE_3D:
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_3D;
      break;

    default:
      status_msg.mode = gps_msgs::msg::GNSSStatus::GNSS_MODE_UNKNOWN;
      break;
  }

  switch (p->status)
  {
    case STATUS_NO_FIX:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_NONE;
      break;

    case STATUS_FIX:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_BASIC;
      break;

    case STATUS_DGPS_FIX:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_DGPS;
      break;

    case STATUS_RTK_FIX:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_RTK_FIXED;
      break;

    case STATUS_RTK_FLT:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_RTK_FLOATING;
      break;

    case STATUS_DR:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_DEAD_RECKONING;
      break;

    case STATUS_GNSSDR:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_GNSS_DEAD_RECKONING;
      break;

    case STATUS_TIME:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_TIME_SURVEYED;
      break;

    case STATUS_SIM:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_SIMULATED;
      break;
    
    case STATUS_PPS_FIX:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_P_Y_CODE;
      break;

    default:
      fix_msg.fix_type = gps_msgs::msg::GNSSFix::FIX_UNKNOWN;
      break;
  }

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
      current_sat->freq_id = map_freq(
        current_sat->gnss_id,
        p->skyview[visible_idx].freqid);
    }
    else
    {
      current_sat->sv_id = gps_msgs::msg::GNSSSatellite::GNSS_INVALID_SV_ID;
      current_sat->gnss_id = gps_msgs::msg::GNSSSatellite::GNSS_ID_INVALID;
      current_sat->freq_id = gps_msgs::msg::GNSSSatellite::SIG_ID_INVALID;
    }

    switch (p->skyview[visible_idx].health)
    {
      case SAT_HEALTH_UNK:
        current_sat->health = gps_msgs::msg::GNSSSatellite::GNSS_SAT_HEALTH_UNKNOWN;
        break;

      case SAT_HEALTH_BAD:
        current_sat->health = gps_msgs::msg::GNSSSatellite::GNSS_SAT_HEALTH_BAD;
        break;

      case SAT_HEALTH_OK:
        current_sat->health = gps_msgs::msg::GNSSSatellite::GNSS_SAT_HEALTH_OK;
        break;

      default:
        current_sat->health = gps_msgs::msg::GNSSSatellite::GNSS_SAT_HEALTH_UNKNOWN;
        break;
    }
    
  }
  return true;
}

int8_t GpsdParserV9::map_freq(const int8_t gnss_id, const int8_t freq_id)
{
  int8_t ret_value = gps_msgs::msg::GNSSSatellite::SIG_ID_UNKNOWN;

  const std::map<int8_t, std::map<int8_t, int8_t>> freq_map =
  {
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_GPS,
      {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L1CA},
      {3, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L2_CL},
      {4, gps_msgs::msg::GNSSSatellite::SIG_ID_GPS_L2_CM}
      }
    },
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_SBAS,
      {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_SBAS_L1CA}
      }
    },
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_GALILEO,
      {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_GALILEO_E1_C},
      {1, gps_msgs::msg::GNSSSatellite::SIG_ID_GALILEO_E1_B},
      {5, gps_msgs::msg::GNSSSatellite::SIG_ID_GALILEO_E5_BL},
      {6, gps_msgs::msg::GNSSSatellite::SIG_ID_GALILEO_E5_BQ}
      }
    },
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_BEIDOU,
      {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_BEIDOU_B1I_D1},
      {1, gps_msgs::msg::GNSSSatellite::SIG_ID_BEIDOU_B1I_D2},
      {2, gps_msgs::msg::GNSSSatellite::SIG_ID_BEIDOU_B2I_D1},
      {3, gps_msgs::msg::GNSSSatellite::SIG_ID_BEIDOU_B2I_D2}
      },
    },
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_QZSS,
      {
      {0, gps_msgs::msg::GNSSSatellite::SIG_ID_QZSS_L1CA},
      {4, gps_msgs::msg::GNSSSatellite::SIG_ID_QZSS_L2_CM},
      {5, gps_msgs::msg::GNSSSatellite::SIG_ID_QZSS_L2_CL}
      }
    },
    {gps_msgs::msg::GNSSSatellite::GNSS_ID_GLONASS,
      {
      {1, gps_msgs::msg::GNSSSatellite::SIG_ID_GLONASS_L1_OF},
      {3, gps_msgs::msg::GNSSSatellite::SIG_ID_GLONASS_L2_OF},
      }
    }
  };

  auto gnss_iter = freq_map.find(gnss_id);

  if (gnss_iter != freq_map.end())
  {
    auto sig_iter = gnss_iter->second.find(freq_id);
    if (sig_iter != gnss_iter->second.end())
    {
      ret_value = sig_iter->second;
    }
  }

  return ret_value;
}
}
