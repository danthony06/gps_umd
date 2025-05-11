#ifndef GPSD_CLIENT_TYPES_HPP_
#define GPSD_CLIENT_TYPES_HPP_

#include <sys/time.h>   // for struct timespec

namespace gpsd_client
{
  struct gnss_data
  {
    timespec online;

  };
}


#endif