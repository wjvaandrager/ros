#include <ros/ros.h>
#include <threemxl/gripper.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/Int32.h>
#include <rosbag/bag.h>
#include <iostream>     // std::cout
#include <cmath>        // std::abs
#include <sensor_msgs/JointState.h>

void DxlROSGripper::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  if (path)
  {
    ROS_INFO("Using shared_serial");
    motor_ = new C3mxlROS(path);
  }
  else
  {
    ROS_INFO("Using direct connection");
    motor_ = new C3mxl();
    
    serial_port_.port_open("/dev/ttyUSB1", LxSerial::RS485_FTDI);
    serial_port_.set_speed(LxSerial::S921600);
    motor_->setSerialPort(&serial_port_);
  }

  motor_->setConfig(config->setID(109));
  motor_->init(false);
  delete config;
  closed = false;
  cur = 0;
  count = 0;
  ros::NodeHandle n;
  //configure visual gripper
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  js.name.resize(7);
  js.position.resize(7);
  js.name[0] ="motor_to_base";
  js.name[1] ="base_to_left_finger";
  js.name[2] ="base_to_right_finger1";
  js.name[3] ="base_to_right_finger2";
  js.name[4] ="left_finger_to_left_fingertip";
  js.name[5] ="right_finger1_to_right_fingertip1";
  js.name[6] ="right_finger2_to_right_fingertip2";

}

void DxlROSGripper::chatterCallback(const std_msgs::Int32::ConstPtr& m)
{
  js.header.stamp = ros::Time::now();
  joint_pub.publish(js); 

  if(!closed && m->data > 20){
     count++;
  }else{
     count = 0;
  }
   
   if( !closed && count > 2){
    //close gripper at full power for half a second and then keep gripping with a smaller force
    motor_->setPWM(-1);
    //dislays simulation of gripper when closing
    while(js.position[1] < 0.7){
      js.header.stamp = ros::Time::now();
      js.position[1] += 0.01;
      js.position[2] -= 0.01;
      js.position[3] -= 0.01;
      js.position[4] += 0.013;
      js.position[5] -= 0.013;
      js.position[6] -= 0.013;
      joint_pub.publish(js);
      usleep(8000);
    }
    usleep(400000);
    motor_->setPWM(-0.3);
    usleep(500000);
    motor_->getState();
    motor_->getStatus();
    cur = motor_->presentCurrent();
    prev = motor_->presentCurrent();
    count = 0;
    
    closed = true;
   }

  motor_->getState();
  motor_->getStatus();
  std::cout << cur << motor_->presentCurrent() << std::endl;
  if(closed && (cur - motor_->presentCurrent()) > 0.015){
      //open gripper for 1 second and then leave it opened
    motor_->setPWM(1);
    //dislays simulation of gripper when opening
    while(js.position[1] > 0.0){
      js.header.stamp = ros::Time::now();
      js.position[1] -= 0.01;
      js.position[2] += 0.01;
      js.position[3] += 0.01;
      js.position[4] -= 0.013;
      js.position[5] += 0.013;
      js.position[6] += 0.013;
      joint_pub.publish(js);
      usleep(6000);
    }
    usleep(500000);
    motor_->setPWM(0);

    count = 0;
    closed = false;
   }
   
   
}

void DxlROSGripper::spin()
{
//open gripper when at beginning
      motor_->setPWM(1);
      usleep(500000);
      motor_->setPWM(0);
    ros::NodeHandle n;

//subscribe to the sensor
    ros::Subscriber sub = n.subscribe<std_msgs::Int32>("chatter", 1, &DxlROSGripper::chatterCallback, this);
    ros::spin();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_gripper");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSGripper dxl_ros_gripper;
  dxl_ros_gripper.init(path);
  dxl_ros_gripper.spin();
  
  return 0;   
} 

