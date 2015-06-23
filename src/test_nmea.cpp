#include <sstream>
#include <iostream>
#include <vector>
#include <string> 
#include <stdlib.h>
#include "nmea.h"

std::string example("$GPGGA,153521.60,4318.93585,N,00124.18890,E,1,08,1.22,323.2,M,48.7,M,,*5A");
std::string ex2("$GPRMC,153520.60,A,4318.93583,N,00124.18892,E,0.059,,150615,,,A*7B");
std::string ex3("$GPGSV,3,3,12,26,58,310,35,27,11,258,09,29,41,058,34,31,51,221,27*72");

  

int main(void)
{
  gps_info_t gps_data;
  clear_gps_data(&gps_data);

  read_nmea_string(&gps_data, example);
  print_gps_data(&gps_data);

  read_nmea_string(&gps_data, ex2);
  print_gps_data(&gps_data);

  read_nmea_string(&gps_data, ex3);
  print_gps_data(&gps_data);
  
  return 1;
}
