// -*-c++-*-

#ifndef GPS_NMEA_DRIVER_H
#define GPS_NMEA_DRIVER_H

#include <string>

using namespace std;

class GpsNmeaDriver
{
public:
  typedef enum {
    INVALID,
    GPS,
    DGPS,
    PPS,
    RTK,
    FLOAT_RTK,
    DEAD_RECKONING,
    MANUAL_INPUT,
    SIMULATION_MODE
  } gps_fixe_t;


  GpsNmeaDriver();
  bool scan(string nmea);
  void print();

  float getLongitude() {return longitude;}
  float getLatitude() {return latitude;}
  float getAltitude() {return altitude;}
  int getFixType() {return fix_type;}
  int getNumSatTracked() {return num_sat_tracked;}
  int getNumSatViewed() {return num_sat_viewed;}
  float getHDOP() {return HDOP;}
  //float getHeading();
  string getFixString();

private:
  void split(vector<string> *result, string str, string delim);
  int checksum(string s);

protected:
  float longitude;
  float latitude;
  float altitude;
  int   fix_type;
  int   num_sat_tracked;
  int   num_sat_viewed; 
  float HDOP;

  float velocity;
  float heading;
  float deviation;
};

#endif
