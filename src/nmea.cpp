#include <sstream>
#include <iostream>
#include <vector>
#include <string> 
#include <stdlib.h>
#include "nmea.h"

//#define __DEBUG__

std::string gps_fix_info[] = {
  "Invalid",
  "GPS fix",
  "DGPS fix",
  "Real Time Kinematic",
  "Float RTK",
  "Dead reckoning",
  "Manual input mode",
  "Simation mode"
};



using namespace std;

//-----------------------------------------------------------------------------
void print_gps_data(gps_info_t *gps_data)
{
  cout << "========================" << endl;
  //cout << "time___________: " <<  gps_data->time << endl;
  cout << "longitude______: " <<  gps_data->longitude << endl;
  cout << "latitude_______: " <<  gps_data->latitude << endl;
  cout << "altitude_______: " <<  gps_data->altitude << endl;
  cout << "fix tyep_______: " <<  gps_fix_info[gps_data->fix_type] << endl;
  cout << "Nb sat tracked_: " <<  gps_data->nb_sat_tracked << endl;
  cout << "Nb sat in view_: " <<  gps_data->nb_sat_viewed << endl;
  cout << "Horizontal DOP_: " <<  gps_data->HDOP << endl;
  cout << "velocity_______: " << gps_data->velocity << endl;
  cout << "heading________: " << gps_data->heading << endl;
  cout << "deviation______: " << gps_data->deviation << endl;
}

//-----------------------------------------------------------------------------
void clear_gps_data(gps_info_t *gps_data)
{
  //gps_data->time = 0;
  gps_data->longitude = 0;
  gps_data->latitude = 0;
  gps_data->altitude = 0;
  gps_data->fix_type = 0;
  gps_data->nb_sat_tracked = 0;
  gps_data->nb_sat_viewed = 0;
  gps_data->HDOP = 0;
  gps_data->velocity = 0;
  gps_data->heading = 0;
  gps_data->deviation = 0;
}

//-----------------------------------------------------------------------------
inline void split(vector<string> *result, string str, string delim)
{
  int pos;
  while( (pos=str.find(delim)) != string::npos) {
    result->push_back(str.substr(0, pos));
    str = str.substr(pos+1);
  }
}

//-----------------------------------------------------------------------------
inline int checksum(string s)
{
  char chksum=0;
  for (string::iterator it = s.begin(), end = s.end(); it != end; ++it)
    chksum ^= (*it);
  return int(chksum)&0xFF;
}

//-----------------------------------------------------------------------------
bool read_nmea_string(gps_info_t *gps_data, string nmea)
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
      //gps_data->time = 0; // TODO convertion
      gps_data->latitude = atof(tokens[2].c_str())*(tokens[3]=="N"?+1:-1);
      gps_data->longitude = atof(tokens[4].c_str())*(tokens[5]=="E"?+1:-1);
      gps_data->fix_type = atoi(tokens[6].c_str());
      gps_data->nb_sat_tracked = atoi(tokens[7].c_str());
      gps_data->HDOP =  atof(tokens[8].c_str());
      gps_data->altitude = atof(tokens[9].c_str());      
      return true;
    }

    if( tokens[0]=="GPRMC") {
      //gps_data->time = 0; //TODO convertion
      gps_data->latitude = atof(tokens[3].c_str())*(tokens[4]=="N"?+1:-1);
      gps_data->longitude = atof(tokens[5].c_str())*(tokens[6]=="E"?+1:-1);
      gps_data->velocity = atof(tokens[7].c_str()) / 1.943844;  // convert knot to m/s
      gps_data->heading = atof(tokens[8].c_str())*3.1415926/180.;  // convert deg to rad
      //gps_data->date = tokens[9]; //todo
      gps_data->deviation = atof(tokens[10].c_str());      
      return true;
    }
    
    if( tokens[0]=="GPGSV") {
      gps_data->nb_sat_viewed = atoi(tokens[3].c_str());
      return true;
    }
  }
  return false;
}
