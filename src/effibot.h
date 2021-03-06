// -*-c++-*-
#ifndef EFFIBOT_H
#define EFFIBOT_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>

#include "conversions.h"

// Effibot
#include <QCoreApplication>
#include <dga-external-component-protocol/VehicleCommunicationListener.hpp>
#include <dga-external-component-protocol/VehicleCommunication.hpp>
#include <dga-external-component-protocol/Common/Types.hpp>


using namespace dga::network;

class Effibot : public VehicleCommunicationListener
{
public:
  typedef enum {
    SECURITY_STOP,
    VELOCITY,
    WAYPOINT,
    IDLE 
  } node_state_t;

  Effibot(ros::NodeHandle node_handle, std::string name);
  ~Effibot();
  
private:
  // ROS callback
  void timerCallback(const ros::TimerEvent&); 
  void waypointCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void commCheckCallback(const std_msgs::Int32 & msg);
  
  // Effibot callback (from Qt Thread)
  void onVehicleConnected();
  void onVehicleDisconnected();

  void onVehicleStateChanged(VehicleState state);
  void onVehicleModeChanged(VehicleMode mode);
  void onVehicleStatusReceived(const VehicleStatus & status);
  void onVehicleOdometryReceived(const VehicleOdometry & odometry);
  void onVehicleMotorCurrentsReceived(const VehicleMotorCurrents & currents);
  void onVehicleLocalizationReceived(const VehicleLocalization & localization);
  void onVehicleImuDataReceived(const ImuData & data);
  void onVehicleGpsDataReceived(const GpsData & data);
  void onVehicleLidarDataReceived(const LidarData & data);
  void onVehicleObstacleMapReceived(const ObstacleMap & data);

  // Function not yet implemented
  void onVehicleConnectionError(const ConnectionError & error) {};
  void onVehicleSendError(const SendError & error);
  void onVehicleWaypointsReceived(int waypointListId);
  void onVehicleWaypointReached(int waypointIndex);
  void onVehicleCommandCancelled();


  // utility function
  std::string getModeString(const VehicleMode &mode);
  std::string getStateString(const VehicleState &state);
  std::string getNodeStateString(node_state_t node_state);


  // ros node and topic
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber comm_check_sub;

  ros::Subscriber goto_goal_sub;      // geometry_msgs::Point
  ros::Publisher goto_feedback_pub;   // std_msgs::Float32 
  ros::Publisher goto_status_pub;     // sdt_msgs::String

  //int running_state;
  

  ros::Publisher  mode_pub;
  ros::Publisher  state_pub;
  ros::Publisher  node_state_pub; 
  ros::Publisher  battery_pub;
  ros::Publisher  odometry_pub;
  ros::Publisher  motor_current_pub;
  ros::Publisher  localization_pub;
  ros::Publisher  pose_pub;
  ros::Publisher  imu_pub;
  ros::Publisher  gps_pub;
  ros::Publisher  laser_pub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Timer loop_timer_;

  // communication test (should receive Int32 on topic comm_check)
  ros::Time last_comm_time;
  double comm_delta_time;
  bool comm_state_valid;

  // Effibot socket communication parameters
  std::string ip_;
  int port_;
  VehicleCommunication communication_;
  bool connected_;

  // Robot status
  float battery_;
  VehicleMode robot_mode_;
  VehicleState robot_state_;
  int bumper_active_;
  int gps_active_;
  
  // Robot property (used for odometry computation)
  double basewidth_;     // lateral distance between wheels
 
  // Relative pose of GPS
  double utm_origin_x;
  double utm_origin_y;
  std::string utm_zone;

  double current_lon_;
  double current_lat_;

  // Odometry
  double odom_prev_date;
  double pose_x;
  double pose_y;
  double pose_theta;

  // Node finite state machine
  node_state_t node_state_;

  // Waypoint control
  int waypoints_ident;

  std::string robot_name;
}; 
#endif
