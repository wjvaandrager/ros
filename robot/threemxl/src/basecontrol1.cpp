#include <ros/ros.h>
#include <threemxl/basecontrol1.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
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

void DxlROSBaseControl::init(char *path)
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
  motor1_->setAcceleration(2,false);
  motor2_->setAcceleration(2,false);
  x_ = 0;
  y_ = 0;
  th_ = 0;
  prevLeft_ = 0;
  prevRight_ = 0;
  wheel_diameter_ = 0.380;
  wheel_base_ = 0.300;
  delete config;
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 5);

}

void DxlROSBaseControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  //haalt de benodigde double uit de message
  double angular = joy->axes[0];
  double gas = joy->axes[9];
  double power = joy->axes[8];
  drive(fabs(gas),power,angular);
}

void DxlROSBaseControl::drive(double gas,double back,double angular){
  double power = 1.5 * gas;
  if(back < 0){
    power = back * 1.5;
  }
  power *=1.5;
  if(angular >= 0){
    motor1_->setSpeed(power*(1 - angular),false);
    motor2_->setSpeed(power,false); 
  } else {
    motor1_->setSpeed(power,false);
    motor2_->setSpeed(power*(1 + angular),false);
  } 
}

void DxlROSBaseControl::steer(double linear,double angular){
}

// reads the current change in wheel and publish as odometry
void DxlROSBaseControl::publishOdometry()
{

	motor1_->getLinearPos();
	motor2_->getLinearPos();

	double left = (motor1_->presentLinearPos() - prevLeft_);
	double right = (motor2_->presentLinearPos() - prevRight_);
	
	if(left == right){
	  x_ += left * cos(th_);
	  y_ += left * sin(th_);
	}else if(left + right == 0){
	  th_-= left/(wheel_base_);
	}else{
	  vx_ = (left + right)/2;
	  double radius = (vx_ * 2 * wheel_base_)/(left -  right);
	  double angle = vx_/(radius);
	  
	  double delta_y = radius * cos(th_) - radius * cos(th_ + angle);
	  double delta_x = radius * sin(th_ + angle) - radius * sin(th_);
//  	  cout << radius << " " << angle/RAD << " " << delta_x << " " << delta_y << endl;
	  x_ += delta_x;
	  y_ += delta_y;
	  th_ -= angle;
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
	current_time_ = ros::Time::now();
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

void DxlROSBaseControl::spin()
{
  ros::Rate loop_rate(25);
        
    ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1, &DxlROSBaseControl::joyCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    publishOdometry();
    loop_rate.sleep();
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_basecontrol");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSBaseControl dxl_ros_basecontrol;
  dxl_ros_basecontrol.init(path);
  dxl_ros_basecontrol.spin();
  
  return 0;   
} 

