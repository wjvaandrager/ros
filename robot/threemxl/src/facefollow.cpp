#include <ros/ros.h>
#include <threemxl/facefollow.h>
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
#include <people_msgs/PositionMeasurement.h>
#include <geometry_msgs/Point.h>

#define _USE_MATH_DEFINES
#define RAD (M_PI/180)
//overbrenging
#define ONLY_WHEEL

using std::cout;
using std::endl;

void DxlROSFaceFollow::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (path)
  {
    ROS_INFO("Using shared_serial");
    motor1_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor1_ = new C3mxl();

    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor1_->setSerialPort(&serial_port_);
  }

  motor1_->setConfig(config->setID(108));
  motor1_->init(false);
  motor1_->set3MxlMode(POSITION_MODE);
  motor1_->setPIDPosition(0.1,0,0,5);
  motor1_->getPos();
  minlim = -M_PI + 1;
  maxlim = M_PI - 1;
  delete config;
 /*
  speed1_pub = n.advertise<std_msgs::Float64>("speed1", 1);
  speed2_pub = n.advertise<std_msgs::Float64>("speed2", 1);
  pos1_pub = n.advertise<std_msgs::Float64>("pos1", 1);
  pos2_pub = n.advertise<std_msgs::Float64>("pos2", 1);
*/
//    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
//    bl_pub_ = nh_.advertise<nav_msgs::Odometry>("/base_link", 10);
}

void DxlROSFaceFollow::pos1(double pos){
  //draait eerst hoek als dat nodig is
  motor1_->getPos();
  //telt de huidige positie bij de gegeven positie op zodat de motor pos vooruit rijdt
  motor1_->setPos(present1() + pos,0.5,false);
  usleep(40000);  
  motor1_->getState();
  while(motor1_->presentSpeed() >0){
    usleep(1000);      
    motor1_->getState();
  }
}

void DxlROSFaceFollow::posEnd(double pos){
  //draait eerst hoek als dat nodig is
  motor1_->getPos();
  //telt de huidige positie bij de gegeven positie op zodat de motor pos vooruit rijdt
  motor1_->setPos(pos,0.5,false);
  usleep(40000);  
  motor1_->getState();
  while(motor1_->presentSpeed() >0){
    usleep(1000);
    motor1_->getState();
  } 
}

double DxlROSFaceFollow::present1(){
  motor1_->getPos();
  return motor1_->presentPos();
}


void DxlROSFaceFollow::msgCallback(const people_msgs::PositionMeasurement::ConstPtr& msg)
{
  double distz = msg->pos.z;
  double distx = msg->pos.x;
  if(fabs(distx)>0){
    double neg = -distx/fabs(distx);
    //berekent hoeken vanaf midden punt naar locatie van eht gezicht
    double pos = neg * acos(distz/sqrt(pow(distx,2) + pow(distz,2)));
    pos /= 2;
    if(fabs(pos) > 0.05){
      if((pos + present1()) > maxlim){
	posEnd(maxlim);
      }else if((pos + present1()) < minlim){
	posEnd(minlim);
      }else{
	pos1(pos);
      }
    }
  }
  
}

void DxlROSFaceFollow::chatCallback(const std_msgs::String::ConstPtr& msg){
  //splitst de message op in stukken
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("\t "));
  //haalt de benodigde double uit de message
  double pos = -atof( strs[1].c_str() )*RAD;
  if((pos + present1()) > maxlim){
    posEnd(maxlim);
  }else if((pos + present1()) < minlim){
    posEnd(minlim);
  }else{
    pos1(pos);
  }
}

void DxlROSFaceFollow::spin()
{
  ros::Rate loop_rate(10);
  cout << minlim << " " << maxlim <<endl;
  ros::Subscriber sub = n.subscribe<people_msgs::PositionMeasurement>("face_detector/people_tracker_measurements", 2, &DxlROSFaceFollow::msgCallback, this);
  ros::Subscriber chat_sub = n.subscribe<std_msgs::String>("/chat",1, &DxlROSFaceFollow::chatCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_facefollow");

  DxlROSFaceFollow dxl_ros_ff;
  dxl_ros_ff.init(NULL);
  dxl_ros_ff.spin();
  
  return 0;   
} 

