#ifndef __BASECONTROL_GOAL_H
#define __BASECONTROL_GOAL_H

#include <ros/ros.h>
#include <stdlib.h> 
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <audioloc/MultiArrayFloat32.h>
using namespace std;

class Sound
{
  protected:
   ros::NodeHandle nh_;   ///< ROS node handle
 
    
  public:
    ros::Subscriber audio_sub;
    ros::Subscriber pause_sub;
    ros::Publisher sound_pub;
    Sound(): nh_("~"){};
    ~Sound()
    {          
	nh_.shutdown();
    }
    void state(const std_msgs::Bool::ConstPtr& msg);
    void calculateLoc(const audioloc::MultiArrayFloat32::ConstPtr& msg);
    void spin();
};



#endif /* __BASECONTROL_GOAL_H */