#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <phidget_ik/phidget_ik.h>
#include <iostream>
#include "std_msgs/Int32.h"

#include <sstream>

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "example");
	ros::NodeHandle n;


        ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000);

        ros::Rate loop_rate(2);
        int count = 0;
	while (ros::ok()){
	   
	  std_msgs::Int32 msg;

    	  std::stringstream ss;
    	  msg.data = count;
  	  chatter_pub.publish(msg);
	  count++;
    	  ros::spinOnce();

    	  loop_rate.sleep();
	}
	return 0;
}


