#include <unistd.h>
#include <iostream>
#include <string>
//#include <boost/algorithm/string.hpp> 
#include <ros/ros.h>
#include <QCoreApplication>
#include <dga-external-component-protocol/VehicleCommunicationListener.hpp>
#include <dga-external-component-protocol/VehicleCommunication.hpp>
#include <dga-external-component-protocol/Common/Types.hpp>

#include "effibot_bridge.h"

using namespace std;

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  ros::init(argc, argv, "effibot_driver");

  if(argc<2) {
    std::cout << "Error sarting effibot driver" << std::endl
	      << "  usage: " << argv[0] << " agent_name ip port goto_enabled" 
	      << std::endl;
    return 0;
  }
  string name = argv[1];
  string ip = argv[2];
  int port = atoi(argv[3]);
  string goto_param = argv[4];

  //bool goto_enabled = (boost::algorithm::to_lower(goto_param) == string("true"));
  bool goto_enabled = (goto_param == string("true"));

  ROS_INFO("Start effibot driver for robot %s on %s:%i", name.c_str(), ip.c_str(), port); 
  if(goto_enabled)
    ROS_INFO("Goto enabled");
  else
    ROS_INFO("Goto disabled");

  EffibotBridge effibot_bridge(name, ip, port, goto_enabled);
  ros::spin();
  std::cout << "Bye bye...\n";
  return 0;
}
