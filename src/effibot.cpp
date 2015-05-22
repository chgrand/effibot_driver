//=============================================================================
//
//
//  TODO : dyanmic_reconfigure : GPS_active, Bumper_active
//  TODO : namespace -->tf
//

#include <unistd.h>
#include <iostream>
#include "effibot.h"

//-----------------------------------------------------------------------------
Effibot::Effibot(ros::NodeHandle node_handle, std::string name) :
  communication_(this),
  nh_(node_handle),
  connected_(false),
  node_state_(SECURITY_STOP),
  waypoints_ident(0),
  robot_name(name)
{
  /*/ TODO origin as param
  utm_origin_x =  370385.000;   // Esperce
  utm_origin_y = 4797265.965;   // Esperce

  utm_origin_x =  398368.00; //398369.61;  // caylus
  utm_origin_y = 4903240.00; //4903128.01;  // caylus

  utm_origin_x =  398369.61;  // caylus
  utm_origin_y = 4903128.01;  // caylus

  utm_origin_x =  376734.39; // Onera
  utm_origin_y = 4825393.60; // Onera

  utm_zone = "31T";
  */

  // get parameters from ROS or use default values
  nh_.param<std::string>("ip", ip_, "127.0.0.1");
  nh_.param<int>("port", port_, 14251);
  nh_.param<double>("basewidth", basewidth_, 0.515); // 515 mm (robot manual)
  nh_.param<int>("state/bumper", bumper_active_, 1);
  nh_.param<int>("state/GPS", gps_active_, 1);
  nh_.param<double>("utm_origin_x", utm_origin_x, 370385.000);   // esperce
  nh_.param<double>("utm_origin_y", utm_origin_y, 4797265.965);  // esperce
  nh_.param<std::string>("utm_zone", utm_zone, "31T");


  // set parmeters to show default values
  //  nh_.setParam("ip", ip_);
  // nh_.setParam("port", port_);
  nh_.setParam("basewidth", basewidth_);
  nh_.setParam("state/bumper", bumper_active_);
  nh_.setParam("state/GPS", gps_active_);
  nh_.setParam("utm_origin_x", utm_origin_x);
  nh_.setParam("utm_origin_y", utm_origin_y);
  nh_.setParam("utm_zone", utm_zone);

  // Publisher (sensors data)
  mode_pub = nh_.advertise<std_msgs::String>("mode",1);
  node_state_pub = nh_.advertise<std_msgs::String>("node_state", 1);
  state_pub = nh_.advertise<std_msgs::String>("state", 1);
  battery_pub = nh_.advertise<std_msgs::Float32>("robot_battery", 1);
  odometry_pub = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  motor_current_pub = nh_.advertise<std_msgs::Float32MultiArray> ("motors_current",1);
  localization_pub = nh_.advertise<sensor_msgs::NavSatFix>("localization",1);
  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose",1); // == localization (utm local)
  imu_pub = nh_.advertise<sensor_msgs::Imu>("imu/data",10);
  //gps_pub = nh_.advertise<std_msgs::String>("gps_nema",10);
  gps_pub = nh_.advertise<nmea_msgs::Sentence>("gps_nema",10);
  laser_pub = nh_.advertise<sensor_msgs::LaserScan>("laser/scan",10);

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

  // main loop callback at 1Hz
  loop_timer_ = nh_.createTimer(ros::Duration(1.0), &Effibot::timerCallback, this);

}


//-----------------------------------------------------------------------------
Effibot::~Effibot()
{
  if(connected_) 
    communication_.disconnectFromVehicle();
}


//-----------------------------------------------------------------------------
void Effibot::timerCallback(const ros::TimerEvent& e)
// FSM main loop
{
  std_msgs::String string_msg;
  std_msgs::Float32 float_msg;  
  
  if(!connected_) {
    ROS_INFO("Try to connect on robot core at %s:%i", ip_.c_str(), port_);
    communication_.connectToVehicle(QString::fromStdString(ip_), port_);
    return;
  }

  std_msgs::String node_state_msg;
  node_state_msg.data = getNodeStateString(node_state_);
  node_state_pub.publish(node_state_msg); 

  // network watchdog check
  ros::Duration delta_time = ros::Time::now() - last_comm_time;
  double comm_delta_time = delta_time.toSec();
  //std::cout << "delta_time : " << comm_delta_time << std::endl;
  if(comm_delta_time>2.0) {
    //ROS_INFO("Node state = SECURITY_STOP");
    communication_.cancelCommand();
    communication_.sendSpeedCommand(VehicleCommand(0,0));  
    node_state_ = SECURITY_STOP;
    return;
  }
  else {
    if(node_state_ == SECURITY_STOP)
      node_state_ = IDLE;
  }
  
  switch(node_state_) {
  case IDLE:
    //ROS_INFO("Node state = IDLE");
    break;

  case VELOCITY:
    //ROS_INFO("Node state = VELOCITY");
    break;
    
  case WAYPOINT:
    //ROS_INFO("Node state = WAYPOINT");
    switch(robot_state_) {
    case StateRunningWaypointsCommand:
      // TODO measure distance
      float_msg.data = 50.0;
      goto_feedback_pub.publish(float_msg);
      break;

    case StateFailureWaypointUnreachable:
      communication_.cancelCommand();
      string_msg.data="Error - Waypoint unreachable";
      goto_status_pub.publish(string_msg);
      float_msg.data = -1.0;
      goto_feedback_pub.publish(float_msg);
      node_state_ = IDLE;
      break;

    case StateWaiting:
      string_msg.data="Success - Waypoint reached";
      goto_status_pub.publish(string_msg);	  
      float_msg.data = 100.0;
      goto_feedback_pub.publish(float_msg);
      node_state_ = IDLE;
      break;
    }	
  }
}


//-----------------------------------------------------------------------------
void Effibot::commCheckCallback(const std_msgs::Int32 & msg)
{
  last_comm_time = ros::Time::now();
  //ROS_INFO("Comm check received!");
}



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
  ROS_INFO("Goto (delta) : %.3f, %.3f", (lon_-current_lon_)*1e5, (lat_-current_lat_)*1e5);
    
  if (node_state_== IDLE) {

    waypoints.id = waypoints_ident++;
    
    point.longitude   = lon_;
    point.latitude    = lat_;

    //point.longitude   = current_lon_;
    //point.latitude    = current_lat_- 5e-5;
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
    //ROS_INFO("cmd_vel = (%.3f, %.3f)", v, w);
    communication_.sendSpeedCommand(VehicleCommand(v,w));  
    //if(node_state_ == VELOCITY) {
    if( (std::abs(v) < std::numeric_limits<double>::epsilon()) &&
        (std::abs(w) < std::numeric_limits<double>::epsilon()))
      node_state_ = IDLE;
    else
      node_state_ = VELOCITY;
  }
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
      return "state_failure";
      
    case StateFailureWaypointUnreachable:
      return "state_failure";
      
    }
  return "state_unknown";
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
  // TODO: optimization of odmetry computation
  //       initial heading ?

  // Date
  double current_date = odometry.date.perception;
  double delta_time = current_date - odom_prev_date;
  odom_prev_date = current_date;

  // Pose update
  double vel_left = (odometry.frontLeftWheelSpeed+
		     odometry.rearLeftWheelSpeed)/2;
  double vel_right = (odometry.frontRightWheelSpeed+
		      odometry.rearRightWheelSpeed/2);
  
  double v_x  = (vel_right+vel_left)/2;
  double v_th = (vel_right-vel_left)/(2*basewidth_);
  
  pose_x += v_x*delta_time*cos(pose_theta);
  pose_y += v_th*delta_time*sin(pose_theta);
  pose_theta += v_th*delta_time;
  
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_theta);

  // publish the transform over tf
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

  //publish the odometry message over ROS
  nav_msgs::Odometry odom_msg;
  //dom_msg.header.stamp = ros::Time(current_date); 
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id  = robot_name+"_base";
  
  odom_msg.pose.pose.position.x = pose_x;
  odom_msg.pose.pose.position.y = pose_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  //odom_msg.pose.covariance = [];

  odom_msg.twist.twist.linear.x = v_x;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = v_th;
  //odom_msg.twist.covariance = [];

  odometry_pub.publish(odom_msg);
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

  //laser_msg.header.stamp = ros::Time(data.date.perception);
  laser_msg.header.stamp = ros::Time::now();
  laser_msg.header.frame_id = robot_name+"_laser";

  // from doc sensor Hokuyo UTM-30LX-EW
  // range = 270 deg
  // 1080 dots/scan
  laser_msg.angle_min = -(135*M_PI/180);
  laser_msg.angle_max = +(135*M_PI/180);
  laser_msg.angle_increment = (270*M_PI/180)/1080;
  laser_msg.time_increment = 0.0;
  laser_msg.scan_time = 0.0;
  laser_msg.range_min = 0.1;
  laser_msg.range_max = 60.0;

  laser_msg.ranges.clear();
  laser_msg.intensities.clear();
  for (int i = 0; i < data.scans.size(); ++i) {
    laser_msg.ranges.push_back(data.scans[i].rho);
    laser_msg.intensities.push_back(0.0);
  }
  
  laser_pub.publish(laser_msg);
}



//-----------------------------------------------------------------------------
void Effibot::onVehicleLocalizationReceived(const VehicleLocalization & localization) 
{
  //pose_pub = nh_.advertise<geometry_msgs::Pose>("pose",1);
  sensor_msgs::NavSatFix localization_msg;

  double lon_ = localization.position.longitude;
  double lat_ = localization.position.latitude;
  current_lon_ = lon_;
  current_lat_ = lat_;
  //ROS_INFO("Pose= (%.6f, %.6f)", lon_, lat_);
  
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

  double utm_x;
  double utm_y;
  std::string zone;
  gps_common::LLtoUTM(lat_, lon_, utm_y, utm_x, zone);
  //std::cout << "GPS Zone=" << zone << std::endl;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();//(localization.date.perception);
  pose_msg.header.frame_id = "odom";
  pose_msg.pose.position.x = utm_x - utm_origin_x;
  pose_msg.pose.position.y = utm_y - utm_origin_y;
  pose_pub.publish(pose_msg);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleImuDataReceived(const ImuData & data) 
{
  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = ros::Time::now();//(data.date.perception);
  imu_msg.header.frame_id = "imu";
  
  imu_msg.linear_acceleration.x = data.accelerationX;
  imu_msg.linear_acceleration.y = data.accelerationY;
  imu_msg.linear_acceleration.z = data.accelerationZ;
  //imu_msg.linear_acceleration_covariance = double[9]; 

  imu_msg.angular_velocity.x = data.angularSpeedX;
  imu_msg.angular_velocity.y = data.angularSpeedY;
  imu_msg.angular_velocity.z = data.angularSpeedZ;
  //imu_msg.angular_velocity_covariance = double[9]; 

  // [TODO] Orientation
  //   geometry_msgs/Quaternion orientation
  //   float64[9] orientation_covariance # Row major about x, y, z axes

  imu_pub.publish(imu_msg);
}

//-----------------------------------------------------------------------------
void Effibot::onVehicleGpsDataReceived(const GpsData & data)
{
  //std_msgs::String msg;
  nmea_msgs::Sentence msg;
  QString nmea_qstring(data.nmeaSentence);
  std::string s = nmea_qstring.toStdString();
  if (!s.empty() && s[s.length()-1] == '\n')
    s.erase(s.length()-1);
  if (!s.empty() && s[s.length()-1] == '\r')
    s.erase(s.length()-1);
  
  msg.header.stamp =  ros::Time::now();//(data.date.perception);
  msg.header.frame_id = robot_name+"_gps";
  msg.sentence = s;
  
  gps_pub.publish(msg);
}


//-----------------------------------------------------------------------------
void Effibot::onVehicleObstacleMapReceived(const ObstacleMap & data)
{
  /*
  std::cout << "Date: " << data.date.perception << " +/- " << data.date.uncertainty << std::endl;
  for (int i = 0; i < data.obstacles.size(); ++i)
    {
      float x = data.obstacles[i].x;
      float y = data.obstacles[i].y;
      std::cout << "Obstacle " << i
		<< " x: " << x << " m" <<
		<< " y: " << y << " m" << std::endl;
    }
  */
}
