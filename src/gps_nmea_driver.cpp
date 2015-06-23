#include <sstream>
#include <iostream>
#include <vector>
#include <string> 
#include <stdlib.h>
#include "gps_nmea_driver.h"

//#define __DEBUG__

using namespace std;

string gps_fix_info[] = {
  "Invalid",
  "GPS fix",
  "DGPS fix",
  "Real Time Kinematic",
  "Float RTK",
  "Dead reckoning",
  "Manual input mode",
  "Simation mode"
};

//-----------------------------------------------------------------------------
string GpsNmeaDriver::getFixString()
{
  return gps_fix_info[fix_type];
}

//-----------------------------------------------------------------------------
void GpsNmeaDriver::print()
{
  cout << "========================" << endl;
  //cout << "time___________: " <<  time << endl;
  cout << "longitude______: " << longitude << endl;
  cout << "latitude_______: " << latitude << endl;
  cout << "altitude_______: " << altitude << endl;
  cout << "fix tyep_______: " << gps_fix_info[fix_type] << endl;
  cout << "Nb sat tracked_: " << num_sat_tracked << endl;
  cout << "Nb sat in view_: " << num_sat_viewed << endl;
  cout << "Horizontal DOP_: " << HDOP << endl;
  cout << "velocity_______: " << velocity << endl;
  cout << "heading________: " << heading << endl;
  cout << "deviation______: " << deviation << endl;
}

//-----------------------------------------------------------------------------
GpsNmeaDriver::GpsNmeaDriver()
{
  //time = 0;
  longitude = 0;
  latitude = 0;
  altitude = 0;
  fix_type = 0;
  num_sat_tracked = 0;
  num_sat_viewed = 0;
  HDOP = 0;
  velocity = 0;
  heading = 0;
  deviation = 0;
}

//-----------------------------------------------------------------------------
inline void GpsNmeaDriver::split(vector<string> *result, string str, string delim)
{
  int pos;
  while( (pos=str.find(delim)) != string::npos) {
    result->push_back(str.substr(0, pos));
    str = str.substr(pos+1);
  }
}

//-----------------------------------------------------------------------------
inline int GpsNmeaDriver::checksum(string s)
{
  char chksum=0;
  for (string::iterator it = s.begin(), end = s.end(); it != end; ++it)
    chksum ^= (*it);
  return int(chksum)&0xFF;
}

//-----------------------------------------------------------------------------
bool GpsNmeaDriver::scan(string nmea)
{
  // Extract data and checksum
  int start = nmea.find('$')+1;
  int end = nmea.find('*');
  string data = nmea.substr(start, (end-start));
  string chksum_str = nmea.substr(end+1,2);
  long int chksum_int = strtol(chksum_str.c_str(), 0, 16);

  #ifdef __DEBUG__
  cout << nmea << endl
       << data << endl
       << hex << chksum_int << dec << endl;
  #endif

  // Compute checksum
  int chksum = checksum(data);
  #ifdef __DEBUG__
  cout << hex << uppercase << chksum << dec << endl;
  #endif
  
  // If checksum is ok
  if (chksum==chksum_int) {
    // Spilt data into tokens
    vector<string> tokens;
    split(&tokens, data, ",");
    #ifdef __DEBUG__
    for (vector<string>::iterator it = tokens.begin(); 
	 it!=tokens.end(); ++it)
      cout << *it << endl;
    cout << data << endl;
    #endif
    
    // Analyse token of a given message type
    if( tokens[0]=="GPGGA") {
      //time = 0; // TODO convertion
      latitude = (float)atof(tokens[2].c_str())*(tokens[3]=="N"?+1:-1);
      longitude = (float)atof(tokens[4].c_str())*(tokens[5]=="E"?+1:-1);
      fix_type = atoi(tokens[6].c_str());
      num_sat_tracked = atoi(tokens[7].c_str());
      HDOP =  (float)atof(tokens[8].c_str());
      altitude = atof(tokens[9].c_str());      
      return true;
    }

    if( tokens[0]=="GPRMC") {
      //time = 0; //TODO convertion
      latitude = atof(tokens[3].c_str())*(tokens[4]=="N"?+1:-1);
      longitude = atof(tokens[5].c_str())*(tokens[6]=="E"?+1:-1);
      velocity = atof(tokens[7].c_str()) / 1.943844;  // convert knot to m/s
      heading = atof(tokens[8].c_str())*3.1415926/180.;  // convert deg to rad
      //date = tokens[9]; //todo
      deviation = atof(tokens[10].c_str());      
      return true;
    }
    
    if( tokens[0]=="GPGSV") {
      num_sat_viewed = atoi(tokens[3].c_str());
      return true;
    }
  }
  return false;
}