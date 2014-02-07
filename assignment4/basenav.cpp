#include <ros/ros.h>
#include <threemxl/basenav.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <stdlib.h> 
#include <iostream>     // std::cout
#include <cmath>        // std::abs
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/algorithm/string.hpp>

#define _USE_MATH_DEFINES
#define RAD (M_PI/180)
#define ONLY_WHEEL

using std::cout;
using std::endl;

void DxlROSBaseNav::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (path)
  {
    ROS_INFO("Using shared_serial");
    motor1_ = new C3mxlROS(path);
    motor2_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor1_ = new C3mxl();
    motor2_ = new C3mxl();  

    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor1_->setSerialPort(&serial_port_);
    motor2_->setSerialPort(&serial_port_);
  }

  motor1_->setConfig(config->setID(106));
  motor1_->init(false);
  motor2_->setConfig(config->setID(107));
  motor2_->init(false);
  motor1_->set3MxlMode(SPEED_MODE);
  motor2_->set3MxlMode(SPEED_MODE);
  motor1_->setPIDPosition(0.1,0,0,5);
  motor2_->setPIDPosition(0.1,0,0,5);
  motor1_->setAcceleration(1,false);
  motor2_->setAcceleration(1,false); 
  x_ = 0;
  y_ = 0;
  th_ = 0;
  prevLeft_ = 0;
  prevRight_ = 0;
  wheel_diameter_ = 0.308;
  wheel_base_ = 0.264;
  delete config;
 /*
  speed1_pub = n.advertise<std_msgs::Float64>("speed1", 1);
  speed2_pub = n.advertise<std_msgs::Float64>("speed2", 1);
  pos1_pub = n.advertise<std_msgs::Float64>("pos1", 1);
  pos2_pub = n.advertise<std_msgs::Float64>("pos2", 1);
*/
   odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
}

//methode voor bereken van kleinste absolute waarde
double DxlROSBaseNav::min(double x,double y){
  if(abs(x) < abs(y)){
    return abs(x);
  }
  return abs(y);
}


void DxlROSBaseNav::setPos(double pos,double wheel){
  if(wheel == 0){
    motor1_->setLinearPos(pos,1,1,false);
    motor2_->setLinearPos(pos,1,1,false);
    usleep(40000);
    motor1_->getState();
    motor2_->getState();
    while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
      publishOdometry();
      motor1_->getState();
      motor2_->getState();
    }
  }
  if(wheel == 1){
    motor1_->setLinearPos(pos,1,1,false);
    usleep(40000);
    motor1_->getState();
    motor2_->getState();
    while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
      publishOdometry();
      motor1_->getState();
      motor2_->getState();
    }
  }
  if(wheel == 2){
    motor2_->setLinearPos(pos,1,1,false);
    usleep(40000);
    motor1_->getState();
    motor2_->getState();
    while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
      publishOdometry();
      motor1_->getState();
      motor2_->getState();
    }
  }
}



// reads the current change in wheel and publish as odometry
void DxlROSBaseNav::publishOdometry()
{

	motor1_->getLinearPos();
	motor2_->getLinearPos();

	double left = (motor1_->presentLinearPos() - prevLeft_)/1.02;
	double right = (motor2_->presentLinearPos() - prevRight_)/1.02;
	
	if(left == right){
	  x_ += left * cos(th_);
	  y_ += left * sin(th_);
	}else if(left + right == 0){
	  th_-= left/(wheel_base_)/1.03;
	}else{
	  vx_ = (left + right)/2;
	  double radius = (vx_ * 2 * wheel_base_)/(left -  right);
	  double angle = vx_/(radius);
	  
	  double delta_y = radius * cos(th_) - radius * cos(th_ + angle);
	  double delta_x = radius * sin(th_ + angle) - radius * sin(th_);
//  	  cout << radius << " " << angle/RAD << " " << delta_x << " " << delta_y << endl;
	  x_ += delta_x;
	  y_ += delta_y;
	  
	  th_ -= angle/1.03;
	}
 	current_time_ = ros::Time::now();
//  	cout << th_/RAD << endl;
	//compute odometry in a typical way given the velocities of the robot
// 	double dt = (current_time_ - last_time_).toSec();
       th_ = fmod(th_,2*M_PI);
	

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

#ifdef ONLY_WHEEL
	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time_;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x_;
	odom_trans.transform.translation.y = y_;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster_.sendTransform(odom_trans);
#endif

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time_;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position
	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.twist.twist.linear.x = vx_;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth_;

	//publish the message
	odom_pub_.publish(odom);
        prevLeft_ = motor1_->presentLinearPos();
	prevRight_ = motor2_->presentLinearPos();
// 	last_time_ = current_time_;
}

void DxlROSBaseNav::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y)
	{
		ROS_WARN("I'm afraid I can't do that, Dave.");
		return;
	}

	if(isnan(msg->linear.x) || isnan(msg->angular.z) || isinf(msg->linear.x) || isinf(msg->angular.z))
	{
		ROS_WARN("I cant travel at infinite speed. Sorry");
		return;
	}

	// Calculate wheel velocities
	double vel_linear  = msg->linear.x/(wheel_diameter_/2);
	double vel_angular = msg->angular.z * (wheel_base_/wheel_diameter_);

	double vel_left    = vel_linear - vel_angular;
	double vel_right   = vel_linear + vel_angular;

	// Actuate
	motor1_->setSpeed(vel_left);
	motor2_->setSpeed(vel_right);

	ROS_DEBUG_STREAM("Base velocity set to [" << vel_left << ", " << vel_right << "]");
}

void DxlROSBaseNav::spin()
{
  ros::Rate loop_rate(50);
        
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &DxlROSBaseNav::velCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    publishOdometry();
    loop_rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_basenav");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSBaseNav dxl_ros_basenav;
  dxl_ros_basenav.init(path);
  dxl_ros_basenav.spin();
  
  return 0;   
} 

