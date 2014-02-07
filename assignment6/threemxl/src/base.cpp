#include <ros/ros.h>
#include <threemxl/base.h>
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

void DxlROSBase::init(char *path)
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
  motor1_->setAcceleration(1,false);
  motor2_->setAcceleration(1,false); 
  x_ = 0;
  y_ = 0;
  th_ = 0;
  prevLeft_ = 0;
  prevRight_ = 0;
  wheel_diameter_ = 0.3;
  wheel_base_ = 0.3;
  delete config;
 /*
  speed1_pub = n.advertise<std_msgs::Float64>("speed1", 1);
  speed2_pub = n.advertise<std_msgs::Float64>("speed2", 1);
  pos1_pub = n.advertise<std_msgs::Float64>("pos1", 1);
  pos2_pub = n.advertise<std_msgs::Float64>("pos2", 1);
*/
   odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
   bl_pub_ = nh_.advertise<nav_msgs::Odometry>("/base_link", 10);
}

//laat de base naar een x en y positie gaan ten opzichte van zichzelf.
void DxlROSBase::corner(double x,double y){
  
  motor1_->getLinearPos();
  motor2_->getLinearPos();
  //draait om als de positie achter zich ligt
  if(y<0){
    turn(180);
  }
  //berekent radius en verschil x en y
  double radius = min(x,y);
  double diff = abs(y) - abs(x);
  double diff1 = abs(x) - abs(y);

  if(abs(x) == radius && diff>0){
    pos(diff,0);
  }
  if(radius >= 0.277){
    //als x de kleinste waarde heeft moet de robot eerst het verschil vooruit rijden

    motor1_->getLinearPos();
    motor2_->getLinearPos();
    //berekent de afstand voor de wielen voor de bocht
    double pos1 = 0.5*M_PI*(radius + 0.277);
    double pos2 = 0.5*M_PI*(radius - 0.277); 
    //voor bocht naar rechts
    if(x >= 0){
      //compenseert het verschil in afstand van de wielen door het ene wiel zachter te laten rijden, zodat beide wielen op hetzelfde tijdstip op de goed plek zijn
      motor1_->setLinearPos(motor1_->presentLinearPos() + pos1,0.6,0.6,false);
      motor2_->setLinearPos(motor2_->presentLinearPos() + pos2,0.6*(pos2/pos1),0.6*(pos2/pos1),false);
      usleep(40000);
      motor1_->getState();
      motor2_->getState();
      while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
	publishOdometry();
	motor1_->getState();
	motor2_->getState();
      }
    }else{
      //voor bocht naar links
      motor1_->setLinearPos(motor1_->presentLinearPos() + pos2,0.6*(pos2/pos1),0.6*(pos2/pos1),false);
      motor2_->setLinearPos(motor2_->presentLinearPos() + pos1,0.6,0.6,false);
      usleep(40000);
      motor1_->getState();
      motor2_->getState();
      while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
	publishOdometry();
	motor1_->getState();
	motor2_->getState();
      }
    }
    //als y de kleinste waarde heeft rijdt de base het verschil nog door na de bocht
  }
  if(abs(y) == radius && diff1>0){
    pos(diff1,0);
  }
}

//stuurt de base aan met een bepaalde snelheid en acceleratie
void DxlROSBase::drive(double v,double w){
	if(w == 0){
		motor1_->setSpeed(v,false);
		motor2_->setSpeed(v,false);  
	}else{
		motor1_->setSpeed(v*w,false);
		motor2_->setSpeed(-v*w,false);		
	}
}

//methode voor bereken van kleinste absolute waarde
double DxlROSBase::min(double x,double y){
  if(abs(x) < abs(y)){
    return abs(x);
  }
  return abs(y);
}

//laat de base een cirkel rijden met een bepaalde radius en rotate geeft aan welke kant op (1 cw, -1 anti-cw)
void DxlROSBase::cirkle(double radius,double rotate){
  //roept vier keer corner aan om een cirkel te maken
  for (int n=0; n<4; n++) {
    corner(rotate * radius,radius);
  }
}

void DxlROSBase::cirkle1(double radius,double cirkles){
  if(radius >= 0.277){
    motor1_->getLinearPos();
    motor2_->getLinearPos();
    //berekent de afstand die de wielen moeten rijden voor het aantal rondjes
    double pos1 = cirkles*2*M_PI*(radius + 0.277);
    double pos2 = cirkles*2*M_PI*(radius - 0.277); 
    //stuurt motoren aan
    motor1_->setLinearPos(motor1_->presentLinearPos() + pos1,0.6,0.6,false);
    motor2_->setLinearPos(motor2_->presentLinearPos() + pos2,0.6*(pos2/pos1),0.6*(pos2/pos1),false);
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

void DxlROSBase::pos(double pos,double angle){
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
  while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
    publishOdometry();
    motor1_->getState();
    motor2_->getState();
  }
}

void DxlROSBase::setPos(double pos,double wheel){
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

//draait de robot een bepaalde hoek om zijn eigen as
void DxlROSBase::turn(double angle){
  angle = angle/1.04;
  motor1_->getLinearPos();
  motor2_->getLinearPos();
  //berekend de positie de wielen voor en achteruit moeten rijden vanaf middelpunt tussen wielen
  double pos1 = motor1_->presentLinearPos() + 2*M_PI*(0.277)*(angle/360);
  double pos2 = motor2_->presentLinearPos() - 2*M_PI*(0.277)*(angle/360);
  //stturt motoren aan
  motor1_->setLinearPos(pos1,0.6,0.6,false);
  motor2_->setLinearPos(pos2,0.6,0.6,false);
  usleep(40000);
  motor1_->getState();
  motor2_->getState();
  while(motor1_->presentSpeed() >0 || motor2_->presentSpeed() >0){
    publishOdometry();
    motor1_->getState();
    motor2_->getState();
  }
}

// reads the current change in wheel and publish as odometry
void DxlROSBase::publishOdometry()
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

//published de posities en snelheden van de motor
void DxlROSBase::publishStates(){
  std_msgs::Float64 s1;
  std_msgs::Float64 s2;
  std_msgs::Float64 p1;
  std_msgs::Float64 p2;  
  
  motor1_->getState();
  motor2_->getState(); 
  motor1_->getLinearPos();
  motor2_->getLinearPos(); 

  s1.data = motor1_->presentSpeed();
  s2.data = motor2_->presentSpeed();
  p1.data = motor1_->presentLinearPos();
  p2.data = motor2_->presentLinearPos();

  speed1_pub.publish(s1);
  speed2_pub.publish(s2);
  pos1_pub.publish(p1);
  pos2_pub.publish(p2);
}

void DxlROSBase::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //splitst de message op in stukken
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("\t "));
  //haalt de benodigde double uit de message
  double x = atof( strs[1].c_str() );
  double y = atof( strs[2].c_str() );
  //stelt error van linearpos bij
   x = 1.04 * x;
  //zoekt de goede methode uit bij de message doet niks bij een niet bestaande methode
  motor1_->getState();
  motor2_->getState();
  if(strs[0] == "stop"){
    motor1_->set3MxlMode(STOP_MODE);
    motor2_->set3MxlMode(STOP_MODE);
  }else if(strs[0] == "drive"){
    if(motor1_->present3MxlMode() == POSITION_MODE){
	motor1_->set3MxlMode(SPEED_MODE);
	motor2_->set3MxlMode(SPEED_MODE);
    }
    drive(x,y);
  }else if(motor1_->presentSpeed() == 0 && motor2_->presentSpeed() == 0){
	  motor1_->set3MxlMode(POSITION_MODE);
	  motor2_->set3MxlMode(POSITION_MODE);     
	  if(strs[0] == "pos"){ 
		pos(x,y);
	  }else if(strs[0] == "setPos"){ 
		setPos(x,y);
	  }else if(strs[0] == "turn"){
		turn(x);
	  }else if(strs[0] == "corner"){ 
		y = 1.04 *y;   
		corner(x,y);
	  }else if(strs[0] == "cirkle"){
		cirkle(x,y);
	  }else if(strs[0] == "cirkle1"){
		cirkle1(x,y);
	  } 
  }else{
    chatterCallback(msg);
  }
}

void DxlROSBase::spin()
{
  ros::Rate loop_rate(50);
        
  ros::Subscriber sub = n.subscribe<std_msgs::String>("chatter", 5, &DxlROSBase::chatterCallback, this);
  while(ros::ok()){
    ros::spinOnce();
    publishOdometry();
    loop_rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_base");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSBase dxl_ros_base;
  dxl_ros_base.init(path);
  dxl_ros_base.spin();
  
  return 0;   
} 

