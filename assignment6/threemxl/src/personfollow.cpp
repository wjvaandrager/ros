#include <ros/ros.h>
#include <threemxl/personfollow.h>
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

void DxlROSPersonFollow::init(char *path)
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
    motor2_ = new C3mxl();
    serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor1_->setSerialPort(&serial_port_);
    motor2_->setSerialPort(&serial_port_);
  }

  motor1_->setConfig(config->setID(106));
  motor1_->init(false);
  motor1_->set3MxlMode(POSITION_MODE);
  motor1_->setPIDPosition(0.1,0,0,5);
  motor2_->setConfig(config->setID(107));
  motor2_->init(false);
  motor2_->set3MxlMode(POSITION_MODE);
  motor2_->setPIDPosition(0.1,0,0,5);
  wheel_diameter_ = 0.3;
  wheel_base_ = 0.3;
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

void DxlROSPersonFollow::pos(double pos,double angle){
  //draait eerst hoek als dat nodig is
  if(angle != 0){
    turn(angle);
  }
  motor1_->getPos();
  motor2_->getPos();
  //telt de huidige positie bij de gegeven positie op zodat de motor pos vooruit rijdt
  motor1_->setPos(present(1) + linearToRad(pos),3,2,false);
  motor2_->setPos(present(2) + linearToRad(pos),3,2,false);
  usleep(40000); 
  motor1_->getState();
  motor2_->getState();
  while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
    motor1_->getState();
    motor2_->getState();
  }
}

//returns pos in rad from a length in meters
double DxlROSPersonFollow::linearToRad(double dist){
  return (2*dist)/wheel_diameter_;
}

//draait de robot een bepaalde hoek om zijn eigen as
void DxlROSPersonFollow::turn(double angle){
  //berekend de positie de wielen voor en achteruit moeten rijden vanaf middelpunt tussen wielen
  double pos1 = present(1) + linearToRad(2*M_PI*wheel_base_*angle);
  double pos2 = present(2) - linearToRad(2*M_PI*wheel_base_*angle);
  //stturt motoren aan
  motor1_->setPos(pos1,3,2,false);
  motor2_->setPos(pos2,3,2,false);
  usleep(40000);
  motor1_->getState();
  motor2_->getState();
  while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
    motor1_->getState();
    motor2_->getState();
  }
}

double DxlROSPersonFollow::present(int motor){
  if(motor == 1){
    motor1_->getPos();
    return motor1_->presentPos();
  }else{
    motor2_->getPos();
    return motor2_->presentPos();
  }
}


void DxlROSPersonFollow::msgCallback(const people_msgs::PositionMeasurement::ConstPtr& msg)
{
  double distz = msg->pos.z;
  double distx = msg->pos.x;
  if(fabs(distx)>0 || fabs(distz - 1.5) > 0){
     //berekent hoeken vanaf midden punt naar locatie van eht gezicht
    double angle = acos(distz/sqrt(pow(distx,2) + pow(distz,2)));
    pos(distz - 1.5,angle);
  }
  
}


void DxlROSPersonFollow::spin()
{
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe<people_msgs::PositionMeasurement>("face_detector/people_tracker_measurements", 2, &DxlROSPersonFollow::msgCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_personfollow");

  DxlROSPersonFollow dxl_ros_pf;
  dxl_ros_pf.init(NULL);
  dxl_ros_pf.spin();
  
  return 0;   
} 

