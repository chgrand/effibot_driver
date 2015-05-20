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

  Effibot effibot_driver(ros::NodeHandle("~"));//, Effibot::Velocity);
  ros::spin();

  std::cout << "Bye bye...\n";
  return 0;
}
