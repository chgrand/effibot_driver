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

    typedef enum {
        GPGGA,
        GPRMC,
        GPGSV,
        UNKNOWN
    } msg_type_t;

  GpsNmeaDriver();
  msg_type_t scan(string nmea);
  void print();

  double getLongitude() {return longitude;}
  double getLatitude() {return latitude;}
  double getAltitude() {return altitude;}
  int getFixType() {return fix_type;}
  int getNumSatTracked() {return num_sat_tracked;}
  int getNumSatViewed() {return num_sat_viewed;}
  double getHDOP() {return HDOP;}
  //double getHeading();
  string getFixString();

private:
  void split(vector<string> *result, string str, string delim);
  int checksum(string s);

protected:
  double longitude;
  double latitude;
  double altitude;
  int   fix_type;
  int   num_sat_tracked;
  int   num_sat_viewed; 
  double HDOP;

  double velocity;
  double heading;
  double deviation;
};

#endif
