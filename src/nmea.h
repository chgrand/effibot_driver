#include <string>

typedef struct{
  //float time;
  float longitude;
  float latitude;
  float altitude;
  int fix_type;
  int nb_sat_tracked;
  float HDOP;

  float velocity;
  float heading;
  float deviation;
  int nb_sat_viewed;
  //std::string date;
} gps_info_t;

enum GPS_FIX {
  INVALID,
  GPS,
  DGPS,
  PPS,
  RTK,
  FLOAT_RTK,
  DEAD_RECKONING,
  MANUAL_INPUT,
  SIMULATION_MODE
};

using namespace std;

void print_gps_data(gps_info_t *gps_data);
void clear_gps_data(gps_info_t *gps_data);
inline void split(vector<string> *result, string str, string delim);
inline int checksum(string s);
bool read_nmea_string(gps_info_t *gps_data, string nmea);
