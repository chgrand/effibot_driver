// -*-c++-*-
#ifndef EFFIBOT_BRIDGE_H
#define EFFIBOT_BRIDGE_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <effibot_msgs/SetConfig.h>

#include "gps_nmea_driver.h"
#include "conversions.h"

// Effibot
#include <QCoreApplication>
#include <dga-external-component-protocol/VehicleCommunicationListener.hpp>
#include <dga-external-component-protocol/VehicleCommunication.hpp>
#include <dga-external-component-protocol/Common/Types.hpp>


using namespace dga::network;

class EffibotBridge : public VehicleCommunicationListener
{
public:
    EffibotBridge(std::string name, std::string ip, int port, bool goto_enabled);
    ~EffibotBridge();

private:
    // ROS callback
    void actionCancelCmd(const std_msgs::Empty& msg);
    void actionCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
    void actionWaypoint(const geometry_msgs::Pose::ConstPtr& msg);
    void connect_loop(const ros::TimerEvent& e);
    bool setConfig(effibot_msgs::SetConfig::Request &req,
                   effibot_msgs::SetConfig::Response &res);

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
    void onVehicleSendError(const SendError & error);
    void onVehicleWaypointsReceived(int waypointListId);
    void onVehicleWaypointReached(int waypointIndex);
    void onVehicleCommandCancelled();

    // Function not yet implemented
    void onVehicleConnectionError(const ConnectionError & error) {};


    // ros node and topic
    ros::NodeHandle nh_;
    ros::Subscriber sub_action_cancel;
    ros::Subscriber sub_action_cmdvel;
    ros::Subscriber sub_action_goto;
    ros::Publisher pub_status;
    ros::Publisher pub_pose;
    ros::Publisher pub_action_status;
    ros::Publisher pub_gps_pose;
    ros::Publisher pub_gps_lla;
    ros::Publisher pub_gps_info;
    ros::Publisher pub_gps_hdop;
    ros::Publisher pub_odometry;
    ros::Publisher pub_motor_current;
    ros::Publisher pub_imu;
    ros::Publisher pub_laser;
    ros::ServiceServer srv_SetConfig;
    ros::Timer connect_loop_timer;

    // Effibot socket communication parameters
    std::string ip_;
    int port_;
    VehicleCommunication communication_;
    bool connected_;

    // Robot status
    VehicleState robot_state_;
    bool bumper_active_;
    bool gps_active_;

    // Robot property (used for odometry computation)
    double basewidth_;     // lateral distance between wheels
    double gps_offset_x;
    double gps_offset_y;

    double lidar_fov;

    // Relative pose of GPS
    double utm_origin_x;
    double utm_origin_y;
    std::string utm_zone;

    // Waypoint control
    int waypoints_ident;
    int waypointNum;
    double pose[2];
    double wp_velocity_;


    /*
    int wp_blocked;
    double twist_Vx;
    double twist_Wz;
    */

    // Gps driver
    GpsNmeaDriver gps_driver;
    std::string robot_name;
};
#endif
