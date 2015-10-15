//=============================================================================
//
//  Effibot Ros Bridge
//
//=============================================================================


#include <unistd.h>
#include <iostream>
#include "effibot_bridge.h"

using namespace std;

//-----------------------------------------------------------------------------
EffibotBridge::EffibotBridge(string name, string ip, int port, bool goto_enabled) :
    communication_(this),
    connected_(false),
    robot_name(name),
    ip_(ip),
    port_(port),
    waypoints_ident(1)
{
    // get parameters from ROS or use default values
    nh_.param<double>("basewidth", basewidth_, 0.515); // 515 mm (robot manual)
    nh_.param<double>("gps/offset/x", gps_offset_x, 0.); // 0 mm (robot manual)
    nh_.param<double>("gps/offset/y", gps_offset_y, -0.165); // 165 mm (robot manual)
    nh_.param<double>("wp_velocity", wp_velocity_, 0.5);      // maximal velocity for waypoint
    nh_.param<double>("lidar_fov", lidar_fov, 170.0);      // lidar angular range
    nh_.param<bool>("state/bumper", bumper_active_, true);
    nh_.param<bool>("state/gps", gps_active_, true);
    nh_.param<double>("/utm_origin_x", utm_origin_x, 0);  // ! global param
    nh_.param<double>("/utm_origin_y", utm_origin_y, 0);
    nh_.param<std::string>("/utm_zone", utm_zone, "");

    // set parmeters to show default values
    nh_.setParam("lidar_fov", lidar_fov);
    nh_.setParam("wp_velocity", wp_velocity_);
    nh_.setParam("basewidth", basewidth_);
    nh_.setParam("state/bumper", bumper_active_);
    nh_.setParam("state/gps", gps_active_);

    // Service config
    srv_SetConfig = nh_.advertiseService("set_config", &EffibotBridge::setConfig, this);

    // Publisher (sensors data)
    pub_status = nh_.advertise<std_msgs::Int32MultiArray>("status",1);
    pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("pose",1);
    pub_action_status = nh_.advertise<std_msgs::Int32>("action/report", 1);
    pub_gps_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps/utm_pose",1);
    pub_gps_lla = nh_.advertise<sensor_msgs::NavSatFix>("gps/lla_pose",1);
    pub_gps_info = nh_.advertise<std_msgs::Int32MultiArray>("gps/info", 1);
    pub_gps_hdop = nh_.advertise<std_msgs::Float32>("gps/hdop", 1);
    pub_odometry = nh_.advertise<nav_msgs::Odometry>("sensors/odometry", 1);
    pub_motor_current = nh_.advertise<std_msgs::Float32MultiArray> ("sensors/motors_current",1);
    pub_imu   = nh_.advertise<sensor_msgs::Imu>("sensors/imu/data",1);
    pub_laser = nh_.advertise<sensor_msgs::LaserScan>("sensors/laser/scan",1);

    // Subscribers
    sub_action_cancel = nh_.subscribe("action/cancel_cmd", 1, &EffibotBridge::actionCancelCmd, this);
    sub_action_cmdvel = nh_.subscribe("action/cmd_vel", 1, &EffibotBridge::actionCmdVel, this);
    sub_action_goto = nh_.subscribe("action/waypoints", 1, &EffibotBridge::actionWaypoint, this);

    // Connection loop callback at 1Hz <--> 1s
    connect_loop_timer = nh_.createTimer(ros::Duration(1), &EffibotBridge::connect_loop, this);

    // main loop callback at 100Hz <--> 10ms
    //main_loop_timer = nh_.createTimer(ros::Duration(0.010), &EffibotBridge::main_loop, this);
}

//-----------------------------------------------------------------------------
EffibotBridge::~EffibotBridge()
{
    if(connected_)
        communication_.disconnectFromVehicle();
}


//============================================================================
//
//     Internal connection logic
//
//============================================================================

//-----------------------------------------------------------------------------
void EffibotBridge::connect_loop(const ros::TimerEvent& e)
{
    if(!connected_) {
        ROS_INFO("Try to connect on robot core at %s:%i", ip_.c_str(), port_);
        communication_.connectToVehicle(QString::fromStdString(ip_), port_);
    }
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleConnected()
{
    connected_ = true;
    ROS_INFO("Vehicle connected");
    communication_.setBumperActive(bumper_active_);
    communication_.setGpsActive(gps_active_);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleDisconnected()
{
    connected_ = false;
    ROS_INFO("Vehicle disconnected");
}


//============================================================================
//
//  Not implemetned in ROS
//
//============================================================================

void EffibotBridge::onVehicleStateChanged(VehicleState state)
{
    //robot_state_ = state;
}
//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleModeChanged(VehicleMode mode)
{
    //robot_mode_ = mode;
}


//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleSendError(const SendError & error)
{
    ROS_INFO("Send error !!!");
}


//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleStatusReceived(const VehicleStatus & status)
{
    /*
    robot_mode_ = status.mode;
    robot_state_ = status.state;
    battery_ = status.batteryLevel;
    */

    robot_state_ = status.state;

    std_msgs::Int32MultiArray array;
    array.data.clear();
    array.data.push_back(status.mode);
    array.data.push_back(status.state);
    array.data.push_back(int(status.batteryLevel*100));
    pub_status.publish(array);

}


//============================================================================
//
//     EffibotBridge action comminucation <-- ROS topic "action/${name}"
//
//============================================================================


void EffibotBridge::actionCancelCmd(const std_msgs::Empty& msg)
{
    if(!connected_)
        return;

    ROS_INFO("Reset action received");
    communication_.cancelCommand();
}


//-----------------------------------------------------------------------------
void EffibotBridge::actionCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(!connected_)
        return;

    double v = msg->linear.x;      // v: vitesse linéaire en m/s
    double w = msg->angular.z;     // omega: vitesse de rotation en rad/s

    // Seuils (2.8, 1.6)
    if(v>2.0) v=2.0; else if(v<-2.0) v=-2;
    if(w>1.0) w=1.0; else if(w<-1.0) w=-1.0;

    switch (robot_state_)
    {
    case StateWaiting:
    case StateRunningSpeedCommand:
        communication_.sendSpeedCommand(VehicleCommand(v,w));
        break;

    case StateRunningWaypointsCommand:
        communication_.cancelCommand();
        communication_.sendSpeedCommand(VehicleCommand(v,w));
        break;

    case StateFailureEmergencyStop:
    case StateFailureCommandNotApplicable:
    case StateFailureWaypointUnreachable:
    default:
        communication_.cancelCommand();
    }
};

//-----------------------------------------------------------------------------
void EffibotBridge::actionWaypoint(const geometry_msgs::Pose::ConstPtr & msg)
{
    if(!connected_)
        return;

    if(robot_state_ != StateWaiting)
    {
        ROS_INFO("Action Waypoint error: robot_state != StateWaiting");
        return;
    }

    double goal_x = msg->position.x;
    double goal_y = msg->position.y;
    double pose_x = pose[0];
    double pose_y = pose[1];
    double dx = goal_x-pose_x;
    double dy = goal_y-pose_y;
    double distance = sqrt(dx*dx+dy*dy);
    double alpha = atan2(dy,dx);
    double Ca = cos(alpha);
    double Sa = sin(alpha);

    ROS_INFO("Goto - Goal = (%.3f, %.3f)", goal_x, goal_y);
    ROS_INFO("Goto - Pose = (%.3f, %.3f)", pose_x, pose_y);
    ROS_INFO("Goto - Distance = %.3f", distance);

    double w_step = 5.0;
    int wp_N = int(ceil(distance/w_step))+1;
    double step = distance/wp_N;
    ROS_INFO("Add %i points", wp_N);

    WaypointList waypoints;

    waypoints.id = waypoints_ident++;
    if(waypoints_ident>255)
        waypoints_ident = 1;

    for(int i=0; i<wp_N;i++)
    {
        double x = pose_x + (i+1)*step*Ca;
        double y = pose_y + (i+1)*step*Sa;
        double utm_x = x + utm_origin_x;
        double utm_y = y + utm_origin_y;
        double lat_;
        double lon_;
        gps_common::UTMtoLL(utm_y, utm_x, "31T", lat_, lon_);
        //ROS_INFO("%.3f, %.3f -- %.6f, %.6f", x , y, lat_, lon_);

        Waypoint point;
        point.longitude   = lon_;
        point.latitude    = lat_;
        point.linearSpeed = wp_velocity_;
        waypoints.append(point);

        // TODO measure initial distance
    }

    waypointNum = wp_N;
    communication_.sendWaypointsCommand(waypoints);
    ROS_INFO("Action Waypoint launched");
};



//============================================================================
//
//    Effibot action callback --> ROS topic "action/report"
//
//============================================================================

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleWaypointsReceived(int waypointListId)
{
    char buffer[128];
    sprintf(buffer, "Waypoint started id=%i", waypointListId);
    std_msgs::String msg;
    msg.data = string(buffer);
    pub_action_status.publish(msg);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleWaypointReached(int waypointIndex)
{
    char buffer[128];
    sprintf(buffer, "Waypoint reached --> %i", waypointIndex);
    std_msgs::String msg;
    msg.data = string(buffer);
    pub_action_status.publish(msg);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleCommandCancelled()
{
    std_msgs::String msg;
    msg.data = "Command cancelled";
    pub_action_status.publish(msg);
}


//=============================================================================
//
//                Effibot bumper and gps config --> ROS service
//
//=============================================================================

bool EffibotBridge::setConfig(effibot_msgs::SetConfig::Request &req,
                        effibot_msgs::SetConfig::Response &res)
{
  gps_active_=req.gps_state;
  bumper_active_ = req.bumper_state;
  nh_.setParam("state/bumper", bumper_active_);
  nh_.setParam("state/GPS", gps_active_);
  communication_.setBumperActive(req.bumper_state);
  communication_.setGpsActive(req.gps_state);
  res.status="Ok";
  ROS_INFO("Set GPS=%i - Bumper=%i", (gps_active_?1:0), (bumper_active_?1:0));
  return true;
}



//=============================================================================
//
//                Effibot sensors callback --> ROS topic
//
//=============================================================================


void EffibotBridge::onVehicleOdometryReceived(const VehicleOdometry & odometry)
{
    // Publish vehicle velocity state based on encoder measurment as odometry message
    // The pose is not estimated here and only the twist is provided

    // Velocity
    double vel_left = (odometry.frontLeftWheelSpeed+
                       odometry.rearLeftWheelSpeed)/2;
    double vel_right = (odometry.frontRightWheelSpeed+
                        odometry.rearRightWheelSpeed/2);

    double v_x  = (vel_right+vel_left)/2;
    double v_th = (vel_right-vel_left)/(2*basewidth_);

    //twist_Vx = v_x;
    //twist_Wz = v_th;

    // publish the odometry message over ROS
    // -------------------------------------
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = robot_name+"_base";

    odom_msg.pose.pose.position.x = 0.0; //pose_x;
    odom_msg.pose.pose.position.y = 0.0; //pose_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0); //pose_theta
    //odom_msg.pose.covariance = [];

    odom_msg.twist.twist.linear.x = v_x;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = v_th;
    //odom_msg.twist.covariance = [];

    pub_odometry.publish(odom_msg);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleMotorCurrentsReceived(const VehicleMotorCurrents & currents)
{
    std_msgs::Float32MultiArray array;
    array.data.clear();
    array.data.push_back(float(currents.frontLeftMotorCurrent));
    array.data.push_back(float(currents.frontRightMotorCurrent));
    array.data.push_back(float(currents.rearLeftMotorCurrent));
    array.data.push_back(float(currents.rearRightMotorCurrent));
    pub_motor_current.publish(array);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleLidarDataReceived(const LidarData & data)
{
    sensor_msgs::LaserScan laser_msg;

    laser_msg.header.stamp = ros::Time::now();
    laser_msg.header.frame_id = robot_name+"_laser";

    // from doc sensor Hokuyo UTM-30LX-EW
    // range = 270 deg
    // 1080 dots/scan --> 0.25°/scan

    //float lidar_fov = 170.0; // TODO rosparam
    int index_min = 540-(int)(lidar_fov*2);
    int index_max = 540+(int)(lidar_fov*2);
    if(index_min<0) index_min = 0;
    if(index_max>data.scans.size()) index_max = data.scans.size();

    laser_msg.angle_min = -(lidar_fov/2*M_PI/180.0);
    laser_msg.angle_max = +(lidar_fov/2*M_PI/180.0);
    laser_msg.angle_increment = (270.0*M_PI/180.0)/1080.0;
    laser_msg.time_increment = 0.0;
    laser_msg.scan_time = 0.0;
    laser_msg.range_min = 0.1;
    laser_msg.range_max = 60.0;

    laser_msg.ranges.clear();
    laser_msg.intensities.clear();
    //for (int i = 0; i < data.scans.size(); ++i) {
    for (int i = index_min; i < index_max; ++i) {
        laser_msg.ranges.push_back(data.scans[i].rho);
        laser_msg.intensities.push_back(0.0);
    }

    pub_laser.publish(laser_msg);
}



//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleLocalizationReceived(const VehicleLocalization & localization)
{
    double lon_ = localization.position.longitude;
    double lat_ = localization.position.latitude;
    double utm_x;
    double utm_y;
    std::string zone;
    gps_common::LLtoUTM(lat_, lon_, utm_y, utm_x, zone);
    pose[0] = utm_x - utm_origin_x - gps_offset_x;
    pose[1] = utm_y - utm_origin_y - gps_offset_y;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = pose[0];
    pose_msg.pose.position.y = pose[1];
    pub_pose.publish(pose_msg);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleImuDataReceived(const ImuData & data)
{
    // Intergrate sensor transformation
    sensor_msgs::Imu imu_msg;

    imu_msg.header.stamp = ros::Time::now();//(data.date.perception);
    imu_msg.header.frame_id = robot_name+"_base";

    imu_msg.angular_velocity.x = -data.angularSpeedX;
    imu_msg.angular_velocity.z = -data.angularSpeedY;
    imu_msg.angular_velocity.y = -data.angularSpeedZ;
    //imu_msg.angular_velocity_covariance = double[9];

    imu_msg.linear_acceleration.x = data.accelerationX;
    imu_msg.linear_acceleration.z = data.accelerationY;
    imu_msg.linear_acceleration.y = data.accelerationZ;
    //imu_msg.linear_acceleration_covariance = double[9];

    // [TODO] Orientation
    //   geometry_msgs/Quaternion orientation
    //   float64[9] orientation_covariance # Row major about x, y, z axes

    // [TODO] add offset on acceleration

    pub_imu.publish(imu_msg);
}

//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleGpsDataReceived(const GpsData & data)
{
    ros::Time date=ros::Time::now();

    QString nmea_qstring(data.nmeaSentence);
    std::string s = nmea_qstring.toStdString();
    GpsNmeaDriver::msg_type_t ret = gps_driver.scan(s);

    std_msgs::Int32MultiArray array;
    array.data.clear();
    array.data.push_back(gps_driver.getFixType());
    array.data.push_back(gps_driver.getNumSatTracked());
    array.data.push_back(gps_driver.getNumSatViewed());
    pub_gps_info.publish(array);

    std_msgs::Float32 msg;
    msg.data = gps_driver.getHDOP();
    pub_gps_hdop.publish(msg);


    if((ret == GpsNmeaDriver::GPGGA)||(ret==GpsNmeaDriver::GPRMC))
    {
        // Converte GPS coordinate to UTM
        double utm_x;
        double utm_y;
        std::string zone;
        gps_common::LLtoUTM(gps_driver.getLatitude(), gps_driver.getLongitude(),
                            utm_y, utm_x, zone);

        // GPS Pose message
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = date;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = utm_x - utm_origin_x - gps_offset_x;
        pose_msg.pose.pose.position.y = utm_y - utm_origin_y - gps_offset_y;
        pose_msg.pose.pose.position.z = gps_driver.getAltitude();

        // Covariance computed from HDOP and nominal variance
        const float nominal_variance = 2.5*2.5;
        for(int i=0;i<36;i++) pose_msg.pose.covariance[i] = 0.0;
        float variance = gps_driver.getHDOP()*nominal_variance/2.;
        pose_msg.pose.covariance[0] = variance;
        pose_msg.pose.covariance[7] = variance;
        pub_gps_pose.publish(pose_msg);

        // GPS LLA message
        sensor_msgs::NavSatFix lla_msg;
        lla_msg.header.stamp = date;
        lla_msg.header.frame_id = "odom";
        lla_msg.status.status = gps_driver.getFixType();
        lla_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        lla_msg.latitude  = gps_driver.getLatitude();
        lla_msg.longitude = gps_driver.getLongitude();
        lla_msg.altitude  = gps_driver.getAltitude();

        // Covariance
        lla_msg.position_covariance_type =
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        for(int i=0;i<9;i++) lla_msg.position_covariance[i] = 0.0;
        lla_msg.position_covariance[0] = variance;
        lla_msg.position_covariance[4] = variance;
        lla_msg.position_covariance[8] = 999;
        pub_gps_lla.publish(lla_msg);

    }
}


//-----------------------------------------------------------------------------
void EffibotBridge::onVehicleObstacleMapReceived(const ObstacleMap & data)
{
}
