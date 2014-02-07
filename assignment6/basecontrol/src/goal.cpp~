#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <basecontrol/goal.h>
#include <stdlib.h>
#include <cmath>  // std::abs
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>
#include <iostream> 
#include <tf/transform_broadcaster.h>
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
#define _USE_MATH_DEFINES
#define RAD (M_PI/180)

using std::cout;
using std::endl;



void Goal::odom(const nav_msgs::Odometry::ConstPtr& msg){
  lastodom.pose = msg->pose;
}

void Goal::followPerson(const std_msgs::String::ConstPtr& msg){
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("\t "));
  double prevAngle = tf::getYaw(lastodom.pose.pose.orientation);
  //haalt de benodigde double uit de message
  double a = atof( strs[1].c_str() )*RAD + prevAngle;
  double dist = atof( strs[0].c_str() );
  double x = dist * cos(a);
  double y = dist * sin(a);
//tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);
//wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){ }
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(a);
  if(dist != 0){
    odom_quat = tf::createQuaternionMsgFromYaw(prevAngle);
  }
  geometry_msgs::PoseStamped goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.header.frame_id = "base_link";
  goal.header.stamp = ros::Time::now();
  
  goal.pose.position.x = lastodom.pose.pose.position.x + x;
  goal.pose.position.x = lastodom.pose.pose.position.y + y;
  goal.pose.orientation = odom_quat;
  goal_pub_.publish(goal);
  sleep(2);
}

void Goal::moveToSound(const std_msgs::String::ConstPtr& msg){
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("\t "));
  //haalt de benodigde double uit de message
  double dist = 10 - log10(atof( strs[1].c_str() ));
  if (dist > lastValue){
    dist = -dist;
  }
  lastValue = dist;
  double a = atof( strs[0].c_str() );
  double x = lastodom.pose.pose.position.x + dist*sin(a);
  double y = lastodom.pose.pose.position.y + dist*cos(a);
  double mapx = x/resolution;
  double mapy = y/resolution;
  double xc = (lastodom.pose.pose.position.x-x)/resolution;
  double yc = (lastodom.pose.pose.position.y-y)/resolution;
  
  if(fabs(xc) > fabs(yc)){
    //wordt -1 of 1 afhankelijk of xc pos of neg was
    xc = xc/fabs(xc);
    yc = yc/fabs(xc);
  }else{
    yc = yc/fabs(yc);
    xc = xc/fabs(yc);    
  }
  cout << a << " " << xc << " " << yc << endl;
  while(map[(int)mapy][(int)mapx] != 0){
     mapy += yc;
     mapx += xc;
  }
//tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);
//wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){ }
  double prevAngle = tf::getYaw(lastodom.pose.pose.orientation);
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(prevAngle + a);
  geometry_msgs::PoseStamped goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.header.frame_id = "base_link";
  goal.header.stamp = ros::Time::now();
  
  goal.pose.position.x = mapx*resolution;
  goal.pose.position.x = mapy*resolution;
  goal.pose.orientation = odom_quat;
  goal_pub_.publish(goal);
  sleep(5);
//   ac.sendGoal(goal);

//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){}
}

void Goal::spin(){
  ros::Rate looprate(20);
  lastValue = 10;
  client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv;
  if(client.call(srv)){
    mp = srv.response.map;
    resolution = mp.info.resolution;
    map = new int*[mp.info.height];
    for(int i = 0;i < (int)mp.info.height; i++){
      map[i] = new int[mp.info.width];
      for(int j =0;j<(int)mp.info.width;j++){
        map[i][j] = mp.data[(i*mp.info.width)+j];
      }
    }    
    cout << "check" << endl;
  }else{
    cout << "fail" << endl;
  }
  odom_sub_ = nh_.subscribe("/odom", 1, &Goal::odom, this);
  sleep(3);
  sound_sub_ = nh_.subscribe("/sound/localisation", 1, &Goal::moveToSound, this);
  person_sub_ = nh_.subscribe("/person", 1, &Goal::followPerson, this);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  while(ros::ok()){
    
    ros::spinOnce();
    looprate.sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  
  Goal g;
  g.spin();
  return 0;
}