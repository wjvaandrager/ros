#include <ros/ros.h>
#include <threemxl/basecontrol.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
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
  motor1_->set3MxlMode(PWM_MODE);
  motor2_->set3MxlMode(PWM_MODE);
  motor1_->setPIDPosition(0.1,0,0,5);
  motor2_->setPIDPosition(0.1,0,0,5);
  delete config;

  speed1_pub = n.advertise<std_msgs::Float64>("speed1", 1);
  speed2_pub = n.advertise<std_msgs::Float64>("speed2", 1);

}

//published de posities en snelheden van de motor
void DxlROSBaseControl::publishStates(){
  std_msgs::Float64 s1;
  std_msgs::Float64 s2;
  motor1_->getState();
  motor2_->getState(); 

  s1.data = motor1_->presentSpeed();
  s2.data = motor2_->presentSpeed();

  speed1_pub.publish(s1);
  speed2_pub.publish(s2);
}

void DxlROSBaseControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  //haalt de benodigde double uit de message
  double linear = joy->axes[1];
  double linear2 = joy->axes[3];
  //stelt error van linearpos bij
  steer(linear*0.5,linear2*0.5);
}

void DxlROSBaseControl::drive(double gas,double back,double angular){
}

void DxlROSBaseControl::steer(double linear,double linear2){
    motor1_->setPWM(linear,false);
    motor2_->setPWM(linear2,false); 
}



void DxlROSBaseControl::spin()
{
  ros::Rate loop_rate(25);
        
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1, &DxlROSBaseControl::joyCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    publishStates();
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

