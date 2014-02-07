#include <ros/ros.h>
#include <threemxl/basekinect.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>
#include <stdlib.h> 
#include <iostream>     // std::cout
#include <cmath>        // std::abs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/algorithm/string.hpp>

#define _USE_MATH_DEFINES
#define RAD (M_PI/180)

using std::cout;
using std::endl;

void DxlROSBaseKinect::init(char *path)
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
  motor1_->set3MxlMode(POSITION_MODE);
  motor2_->set3MxlMode(POSITION_MODE);
  motor1_->setPIDPosition(0.1,0,0,5);
  motor2_->setPIDPosition(0.1,0,0,5);
  delete config;

  speed1_pub = n.advertise<std_msgs::Float64>("speed1", 1);
  speed2_pub = n.advertise<std_msgs::Float64>("speed2", 1);
  dist_pub = n.advertise<std_msgs::Float64>("dist", 1);
  angle_pub = n.advertise<std_msgs::Float64>("angle", 1);
  pos_pub = n.advertise<std_msgs::Float64>("pos", 1);
  
}

void DxlROSBaseKinect::pos(double pos,double angle){
  //draait eerst hoek als dat nodig is
  if(angle != 0){
    turn(angle);
  }
  motor1_->getLinearPos();
  motor2_->getLinearPos();
  //telt de huidige positie bij de gegeven positie op zodat de motor pos vooruit rijdt
  motor1_->setLinearPos(motor1_->presentLinearPos() + pos,0.6,0.6,false);
  motor2_->setLinearPos(motor2_->presentLinearPos() + pos,0.6,0.6,false); 
  usleep(40000);
  motor1_->getState();
  motor2_->getState(); 
  while(motor1_->presentSpeed() != 0 || motor2_->presentSpeed() != 0){
    publishStates(); 
  }
}

//draait de robot een bepaalde hoek om zijn eigen as
void DxlROSBaseKinect::turn(double angle){

  motor1_->getLinearPos();
  motor2_->getLinearPos();
  //berekend de positie de wielen voor en achteruit moeten rijden vanaf middelpunt tussen wielen
  double pos1 = motor1_->presentLinearPos() + 2*M_PI*(0.2644)*(angle/360);
  double pos2 = motor2_->presentLinearPos() - 2*M_PI*(0.2644)*(angle/360);
  //stturt motoren aan
  motor1_->setLinearPos(pos1,0.6,0.6,false);
  motor2_->setLinearPos(pos2,0.6,0.6,false);
  usleep(40000);
  motor1_->getState();
  motor2_->getState(); 
  while(motor1_->presentSpeed() != 0 || motor2_->presentSpeed() != 0){
    publishStates(); 
  }
}

//published de posities en snelheden van de motor
void DxlROSBaseKinect::publishStates(){
  std_msgs::Float64 s1;
  std_msgs::Float64 s2;
  
  motor1_->getState();
  motor2_->getState(); 

  s1.data = motor1_->presentSpeed();
  s2.data = motor2_->presentSpeed();

  speed1_pub.publish(s1);
  speed2_pub.publish(s2);
}

void DxlROSBaseKinect::kinectCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   motor1_->getState();
   motor2_->getState();
   if(motor1_->presentSpeed() == 0 && motor2_->presentSpeed() == 0){
    //factor is afstand tot muur in meters
    double distance = 1;
    //afstand van laserscan waardes op -5 en 0 graden
    double angle = 0;
    double count = 0;
    double dist = 0;
    double dir = 1;
    for( int i = 5; i< 60;i+=5){
       if(msg->ranges[i] < 6 && msg->ranges[i+5] < 6){
	 double length = cosRuleLength(msg->ranges[i],msg->ranges[i+5],2.5);
	 double a = cosRuleAngle(msg->ranges[i],length,msg->ranges[i+5]);
	 angle += a + (60-i)/2 -90;
	 dist += msg->ranges[i] * sin(a*RAD);
	 count++;
       }
    }
    if(angle < 0){
      angle = fabs(angle);
      dir = -1;
    }
    for( int i = 60; i< 111;i+=5){
       if(msg->ranges[i] < 6 && msg->ranges[i+5] < 6){
	 double length = cosRuleLength(msg->ranges[i],msg->ranges[i+5],2.5);
	 double a = cosRuleAngle(msg->ranges[i+5],length,msg->ranges[i]);
	 angle += fabs(a  + (i - 55)/2 -90);
	 dist += msg->ranges[i+5] * sin(a*RAD);
	 count++;
       }
    }
    //voor correcte waarden van laserscan bereken hoek muur en afstand
    angle =  angle/count;
    dist = dist/count;

    std_msgs::Float64 p;
    std_msgs::Float64 a;
    std_msgs::Float64 d;
    
    cout << "Angle: " << angle << "Distance: " << dist << endl;
    if(fabs(angle) > 3 || fabs(dist - distance) > 0.03){
      //berekent positie die de base moet rijden na draaien
      //maal 1.04 voor compensatie en +0.02 voor calibratie kinect
      p.data = dist - distance;
      a.data = dir*angle;
      d.data = dist;
      
      pos_pub.publish(p);
      angle_pub.publish(a);
      dist_pub.publish(d);
  
      pos((dist -distance),dir*angle);
    }else{
      motor1_->setPWM(0,false);
      motor2_->setPWM(0,false);    
    }
  }
}

double DxlROSBaseKinect::angleBase(double l1,double l2,double angle){
  angle = angle/57.2957795;
  double rad = 0;
  //lengte tussen l1 en l2
  double l3 = sqrt(pow(l1,2) + pow(l2,2) - 2*l2*l1*cos(angle));
  //hoek ten opzichte van muur
  rad = 0.5*M_PI - asin((l1*sin(angle))/l3);
  if(l2>l1){
    rad = -rad; 
  }
  return rad*57.2957795;
}


double DxlROSBaseKinect::cosRuleLength(double l1,double l2,double angle){
  return sqrt(pow(l1,2) + pow(l2,2) - 2*l1*l2*cos(angle*RAD));
}

//l3 is overstaande zijde
double DxlROSBaseKinect::cosRuleAngle(double l1,double l2,double l3){
  return acos((pow(l1,2) + pow(l2,2) - pow(l3,2))/(2*l1*l2))/RAD;
}

double DxlROSBaseKinect::distBase(double l1,double l2,double angle){
  double l3 = cosRuleLength(l1,l2,angle);
//   return fabs(90 - angleMax - cosRuleAngle(l1,l3,l2));
  return l1*sin(cosRuleAngle(l1,l3,l2)*RAD);
}

void DxlROSBaseKinect::spin()
{
  ros::Rate loop_rate(100);
        
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &DxlROSBaseKinect::kinectCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    publishStates();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "dxl_ros_basekinect");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSBaseKinect dxl_ros_basekinect;
  
   dxl_ros_basekinect.init(path);
  dxl_ros_basekinect.spin();

  return 0;   
} 

