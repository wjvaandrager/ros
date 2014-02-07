#ifndef __BASECONTROL_ARRAY_H
#define __BASECONTROL_ARRAY_H

#include <ros/ros.h>
#include <stdlib.h> 
#include <stdio.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
using namespace std;

class Array
{
  protected:
   ros::NodeHandle nh_;   ///< ROS node handle
 
    
  public:
    ros::Subscriber sound_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher array_pub_;
    Array(): nh_("~"){};
    ~Array()
    {          
	nh_.shutdown();
    }
    

    void spin();
};

#endif /* __BASECONTROL_ARRAY_H */