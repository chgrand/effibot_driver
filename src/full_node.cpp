#include <unistd.h>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <QCoreApplication>
#include <dga-external-component-protocol/VehicleCommunicationListener.hpp>
#include <dga-external-component-protocol/VehicleCommunication.hpp>
#include <dga-external-component-protocol/Common/Types.hpp>

#include "effibot.h"

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);
  ros::init(argc, argv, "effibot_driver");

  if(argc<2) {
    std::cout << "Error sarting effibot driver" << std::endl
	      << "  usage: " << argv[0] << " agent_name" << std::endl;
    return 0;
  }
  Effibot effibot_driver(ros::NodeHandle(""), "effibot1");//, Effibot::Velocity);
  ros::spin();

  std::cout << "Bye bye...\n";
  return 0;
}
