#ifndef __BASECONTROL_GOAL_H
#define __BASECONTROL_GOAL_H

#include <ros/ros.h>
#include <stdlib.h> 
#include <stdio.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
using namespace std;

class Goal
{
  protected:
   ros::NodeHandle nh_;   ///< ROS node handle
 
    
  public:
    nav_msgs::Odometry lastodom;
    nav_msgs::OccupancyGrid mp;
    double resolution;
    double lastValue;
    int **map;
    ros::Subscriber sound_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber person_sub_;
    ros::Publisher goal_pub_;
    ros::ServiceClient client;
    Goal(): nh_("~"){};
    ~Goal()
    {          
	nh_.shutdown();
    }
    

    void spin();
    void odom(const nav_msgs::Odometry::ConstPtr& msg);
    void moveToSound(const std_msgs::String::ConstPtr& msg);
    void followPerson(const std_msgs::String::ConstPtr& msg);
};



#endif /* __BASECONTROL_GOAL_H */