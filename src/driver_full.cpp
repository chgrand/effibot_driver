//=============================================================================
//
//
//  TODO : dyanmic_reconfigure : GPS_active, Bumper_active
//  TODO : namespace -->tf
//

#include <unistd.h>
#include <iostream>
#include "driver_full.h"

using namespace std;

//-----------------------------------------------------------------------------
Effibot::Effibot(string name, string ip, int port) :
    communication_(this),
    connected_(false),
    node_state_(SECURITY_STOP),
    robot_name(name),
    ip_(ip),
    port_(port),
    waypoints_ident(0)
{
    // get parameters from ROS or use default values
    nh_.param<double>("basewidth", basewidth_, 0.515); // 515 mm (robot manual)
    nh_.param<double>("gps/offset/x", gps_offset_x, 0.); // 0 mm (robot manual)
    nh_.param<double>("gps/offset/y", gps_offset_y, -0.165); // 165 mm (robot manual)
    nh_.param<int>("state/bumper", bumper_active_, 1);
    nh_.param<int>("state/GPS", gps_active_, 1);

    // Global parameters
    nh_.param<double>("/utm_origin_x", utm_origin_x, 0);
    nh_.param<double>("/utm_origin_y", utm_origin_y, 0);
    nh_.param<std::string>("/utm_zone", utm_zone, "");


    // set parmeters to show default values
    nh_.setParam("basewidth", basewidth_);
    nh_.setParam("state/bumper", bumper_active_);
    nh_.setParam("state/GPS", gps_active_);

    // Publisher (sensors data)
    mode_pub = nh_.advertise<std_msgs::String>("mode",1);
    node_state_pub = nh_.advertise<std_msgs::String>("node_state", 1);
    state_pub = nh_.advertise<std_msgs::String>("state", 1);
    battery_pub = nh_.advertise<std_msgs::Float32>("robot_battery", 1);
    odometry_pub = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
    motor_current_pub = nh_.advertise<std_msgs::Float32MultiArray> ("motors_current",1);
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("effibox/utm_pose",1);
    imu_pub = nh_.advertise<sensor_msgs::Imu>("imu/data",1);
    laser_pub = nh_.advertise<sensor_msgs::LaserScan>("laser/scan",1);
    gps_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps/utm_pose",1);
    gps_info_pub = nh_.advertise<std_msgs::Int32MultiArray>("gps/info", 1);
    gps_hdop_pub = nh_.advertise<std_msgs::Float32>("gps/hdop", 1);


    // Subsribe to control input
    cmd_vel_sub = nh_.subscribe("cmd_vel", 1, &Effibot::velocityCallback, this);

  // Waypoint function
    goto_goal_sub = nh_.subscribe("goto/goal", 1, &Effibot::waypointCallback, this);
    goto_feedback_pub = nh_.advertise<std_msgs::Float32>("goto/feedback", 1);
    goto_status_pub = nh_.advertise<std_msgs::String>("goto/status", 1);

    // Topic should receive data at 1Hz over the network to enable motion
    // could be used as emergency stop
    comm_check_sub = nh_.subscribe("comm_check", 1, &Effibot::commCheckCallback, this);
    last_comm_time = ros::Time::now();

    // main loop callback at 100Hz <--> 10ms
    main_loop_timer = nh_.createTimer(ros::Duration(0.010), &Effibot::main_loop, this);

    // main loop callback at 1Hz <--> 1s
    connect_loop_timer = nh_.createTimer(ros::Duration(1), &Effibot::connect_loop, this);

    velocity_linear = 0.0;
    velocity_angular = 0.0;
}


//-----------------------------------------------------------------------------
Effibot::~Effibot()
{
    if(connected_)
        communication_.disconnectFromVehicle();
}



//-----------------------------------------------------------------------------
void Effibot::connect_loop(const ros::TimerEvent& e)
{
    if(!connected_) {
        ROS_INFO("Try to connect on robot core at %s:%i", ip_.c_str(), port_);
        communication_.connectToVehicle(QString::fromStdString(ip_), port_);
    }
}

//-----------------------------------------------------------------------------
void Effibot::main_loop(const ros::TimerEvent& e)
{
    std_msgs::String string_msg;
    std_msgs::Float32 float_msg;

    if(connected_) {
        // publish node_state
        string_msg.data = getNodeStateString(node_state_);
        node_state_pub.publish(string_msg);

        // network watchdog check
        ros::Duration delta_time = ros::Time::now() - last_comm_time;
        double comm_delta_time = delta_time.toSec();
        if(comm_delta_time>1.2) {
            velocity_linear = 0.0;
            velocity_angular = 0.0;
	    communication_.cancelCommand();
            //communication_.sendSpeedCommand(VehicleCommand(0,0));
            node_state_ = SECURITY_STOP;
            return;
        }
        else {
            if(node_state_ == SECURITY_STOP)
                node_state_ = IDLE;
        }

        switch(node_state_) {
        case IDLE:
	  communication_.sendSpeedCommand(VehicleCommand(0*velocity_linear, 0*velocity_angular));
	  break;

        case VELOCITY:
	  communication_.sendSpeedCommand(VehicleCommand(velocity_linear, velocity_angular));
	  break;

	case WAYPOINT:
	  break;
        }
    }
}

//-----------------------------------------------------------------------------
void Effibot::commCheckCallback(const std_msgs::Int32 & msg)
{
    last_comm_time = ros::Time::now();
}



//-----------------------------------------------------------------------------
void Effibot::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double v = msg->linear.x;      // v: vitesse linéaire en m/s
    double w = msg->angular.z;     // omega: vitesse de rotation en rad/s

    // Seuils
    if(v>2.8) v=2.8;
    else if(v<-2.8) v=-2.8;
    if(w>1.6) w=1.6;
    else if(w<-1.6) w=-1.6;

    if(node_state_ != SECURITY_STOP) {
      if(node_state_ == WAYPOINT)
	    communication_.cancelCommand();
	
      velocity_linear = v;
      velocity_angular = w;

        //ROS_INFO("cmd_vel = (%.3f, %.3f)", v, w);
        //if(node_state_ == VELOCITY) {
        if( (std::abs(v) < std::numeric_limits<double>::epsilon()) &&
            (std::abs(w) < std::numeric_limits<double>::epsilon()))
            node_state_ = IDLE;
        else
            node_state_ = VELOCITY;
    }
};

//-----------------------------------------------------------------------------
void Effibot::waypointCallback(const geometry_msgs::Pose::ConstPtr & msg)
{
  WaypointList waypoints;
  Waypoint point;

  double x_goal = msg->position.x;
  double y_goal = msg->position.y;
  double utm_x = x_goal + utm_origin_x;
  double utm_y = y_goal + utm_origin_y;
  double lat_;
  double lon_;
  gps_common::UTMtoLL(utm_y, utm_x, "31T", lat_, lon_);

  ROS_INFO("Receive Goto(%.3f, %.3f)", x_goal, y_goal);
  ROS_INFO("Goto (utm): %.3f, %.3f", utm_x, utm_y);
  ROS_INFO("Goto (gps): %.6f, %.6f", lon_, lat_);
  //ROS_INFO("Goto (delta) : %.3f, %.3f", (lon_-current_lon_)*1e5, (lat_-current_lat_)*1e5);

  if (node_state_== IDLE) {

    waypoints.id = waypoints_ident++;
    if(waypoints_ident>255)
        waypoints_ident = 1;

    point.longitude   = lon_;
    point.latitude    = lat_;
    point.linearSpeed = 0.5;  // TODO PARAM
    waypoints.append(point);

    // TODO measure initial distance

    communication_.sendWaypointsCommand(waypoints);
    node_state_ = WAYPOINT;
    ROS_INFO("Action launched");
  }
  else
    ROS_INFO("Action not validated");
};






//-----------------------------------------------------------------------------
void Effibot::onVehicleSendError(const SendError & error)
{
    std::cout << "Send error !!!\n";
}


//-----------------------------------------------------------------------------
void Effibot::onVehicleWaypointsReceived(int waypointListId)
{
    std::cout << "Points de passage de la sequence " << waypointListId
              << " pris en compte" << std::endl;
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleWaypointReached(int waypointIndex)
{
    std::cout << "Point de passage " << waypointIndex << " atteint." << std::endl;
    node_state_ = IDLE;
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleCommandCancelled()
{
    std::cout << "Commande cancelled !" << std::endl;
}


//-----------------------------------------------------------------------------
std::string Effibot::getNodeStateString(node_state_t node_state)
{
    switch(node_state) {
    case SECURITY_STOP:
        return "Security_stop";
    case IDLE:
        return "Idle";
    case VELOCITY:
        return("Velocity");
    case WAYPOINT:
        return("Waypoint");
    default:
        return "Unknown";
    }
}

//-----------------------------------------------------------------------------
std::string Effibot::getModeString(const VehicleMode &mode)
{
    switch (mode)
    {
    case ModeManual:
        return "mode_manuel";

    case ModeExternalComponent:
        return "mode_pilote";

    }
    return "mode_unknown";
}


//-----------------------------------------------------------------------------
std::string Effibot::getStateString(const VehicleState &state)
{
    switch (state)
    {
    case StateWaiting:
        return "state_waiting";

    case StateRunningSpeedCommand:
        return "state_running_Vel";

    case StateRunningWaypointsCommand:
        return "state_running_WP";

    case StateFailureEmergencyStop:
        return "state_emergency_stop";

    case StateFailureCommandNotApplicable:
        return "state_failure_Command_Not_Applicable";

    case StateFailureWaypointUnreachable:
        return "state_failure_Waypoint_Unreachable";

    }
    char buffer[256];
    sprintf(buffer,"state_unknown = %i", state);
    return std::string(buffer);
}


//-----------------------------------------------------------------------------
void Effibot::onVehicleConnected()
{
    connected_ = true;
    ROS_INFO("Vehicle connected");
    if(bumper_active_)
        communication_.setBumperActive(true);
    else
        communication_.setBumperActive(false);
    if(gps_active_)
        communication_.setGpsActive(true);
    else
        communication_.setGpsActive(false);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleDisconnected()
{
    connected_ = false;
    ROS_INFO("Vehicle disconnected");
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleStateChanged(VehicleState state)
{
    robot_state_ = state;
}
//-----------------------------------------------------------------------------
void Effibot::onVehicleModeChanged(VehicleMode mode)
{
    robot_mode_ = mode;
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleStatusReceived(const VehicleStatus & status)
{
    ///double date = status.date.perception;
    robot_mode_ = status.mode;
    robot_state_ = status.state;
    battery_ = status.batteryLevel;

    std_msgs::Float32 battery_msg;
    battery_msg.data = battery_;
    battery_pub.publish(battery_msg);

    std_msgs::String state_msg;
    state_msg.data = getStateString(robot_state_);
    state_pub.publish(state_msg);

    std_msgs::String mode_msg;
    mode_msg.data = getModeString(robot_mode_);
    mode_pub.publish(mode_msg);
};

//-----------------------------------------------------------------------------
void Effibot::onVehicleOdometryReceived(const VehicleOdometry & odometry)
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


    /*
      pose_x += v_x*delta_time*cos(pose_theta);
      pose_y += v_th*delta_time*sin(pose_theta);
      pose_theta += v_th*delta_time;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_theta);
    */

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

    odometry_pub.publish(odom_msg);


    // Publish the transform over tf
    // -----------------------------
    /*
      geometry_msgs::TransformStamped odom_trans;
      //odom_trans.header.stamp = ros::Time(current_date);
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = robot_name+"_base";

      odom_trans.transform.translation.x = pose_x;
      odom_trans.transform.translation.y = pose_y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);
    */
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleMotorCurrentsReceived(const VehicleMotorCurrents & currents)
{
    //double date = data.date.perception;

    std_msgs::Float32MultiArray array;
    array.data.clear();
    array.data.push_back(float(currents.frontLeftMotorCurrent));
    array.data.push_back(float(currents.frontRightMotorCurrent));
    array.data.push_back(float(currents.rearLeftMotorCurrent));
    array.data.push_back(float(currents.rearRightMotorCurrent));

    motor_current_pub.publish(array);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleLidarDataReceived(const LidarData & data)
{
    sensor_msgs::LaserScan laser_msg;

    laser_msg.header.stamp = ros::Time::now();
    laser_msg.header.frame_id = robot_name+"_laser";

    // from doc sensor Hokuyo UTM-30LX-EW
    // range = 270 deg
    // 1080 dots/scan --> 0.25°/scan

    float lidar_fov = 170.0;
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

    laser_pub.publish(laser_msg);
}



//-----------------------------------------------------------------------------
void Effibot::onVehicleLocalizationReceived(const VehicleLocalization & localization)
{
    sensor_msgs::NavSatFix localization_msg;

    double lon_ = localization.position.longitude;
    double lat_ = localization.position.latitude;
    //current_lon_ = lon_;
    //current_lat_ = lat_;
    //ROS_INFO("Pose= (%.6f, %.6f)", lon_, lat_);

    /*
      localization_msg.header.stamp = ros::Time::now();//(localization.date.perception);
      localization_msg.header.frame_id = "odom";
      localization_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      localization_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
      localization_msg.latitude  = lat_;  // deg
      localization_msg.longitude = lon_; // deg
      localization_msg.altitude  = localization.position.altitude;  // m

      // [TODO]: covariance
      localization_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      //localization_msg.position_covariance=[];

      // TODO : integrate GPS velocity data
      //    - localization.velocity.linearSpeed  // m/s
      //    - localization.velocity.course  // rad

      localization_pub.publish(localization_msg);
    */

    double utm_x;
    double utm_y;
    std::string zone;
    gps_common::LLtoUTM(lat_, lon_, utm_y, utm_x, zone);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = utm_x - utm_origin_x - gps_offset_x;
    pose_msg.pose.position.y = utm_y - utm_origin_y - gps_offset_y;
    pose_pub.publish(pose_msg);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleImuDataReceived(const ImuData & data)
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

    imu_pub.publish(imu_msg);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleGpsDataReceived(const GpsData & data)
{
    ros::Time date=ros::Time::now();

    QString nmea_qstring(data.nmeaSentence);
    std::string s = nmea_qstring.toStdString();
    //std::cout << s << std::endl;
    //std:: cout << "Decode = " << gps_driver.scan(s) << std::endl;
    //gps_driver.print();

    gps_driver.scan(s);

    std_msgs::Int32MultiArray array;
    array.data.clear();
    array.data.push_back(gps_driver.getFixType());
    array.data.push_back(gps_driver.getNumSatTracked());
    array.data.push_back(gps_driver.getNumSatViewed());
    gps_info_pub.publish(array);

    std_msgs::Float32 msg;
    msg.data = gps_driver.getHDOP();
    gps_hdop_pub.publish(msg);


    if(gps_driver.getFixType() != 0) {
        // Converte GPS coordinate to UTM
        double utm_x;
        double utm_y;
        std::string zone;
        gps_common::LLtoUTM(gps_driver.getLatitude(), gps_driver.getLongitude(), utm_y, utm_x, zone);

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

        gps_pub.publish(pose_msg);
    }
}


//-----------------------------------------------------------------------------
void Effibot::onVehicleObstacleMapReceived(const ObstacleMap & data)
{
}
