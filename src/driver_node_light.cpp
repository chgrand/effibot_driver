#include <unistd.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <QCoreApplication>
#include <dga-external-component-protocol/VehicleCommunicationListener.hpp>
#include <dga-external-component-protocol/VehicleCommunication.hpp>
#include <dga-external-component-protocol/Common/Types.hpp>

#include "driver_light.h"

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  ros::init(argc, argv, "effibot_driver");

  if(argc<2) {
    std::cout << "Error sarting effibot driver" << std::endl
	      << "  usage: " << argv[0] << " agent_name ip port" 
	      << std::endl;
    return 0;
  }
  string name = argv[1];
  string ip = argv[2];
  int port = atoi(argv[3]);

  ROS_LOG("Start effibot driver for robot %s on %s:%i", name, ip, port) 
  Effibot effibot_driver(name, ip, port);
  ros::spin();

  std::cout << "Bye bye...\n";
  return 0;
}
