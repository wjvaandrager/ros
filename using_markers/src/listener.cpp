#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <visualization_msgs/Marker.h>
#include <using_markers/listener.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Listener::chatterCallback(const std_msgs::Int32::ConstPtr& msg){

    
    if(msg->data < 5){
      //make object invisible if no object is close 
      marker.color.a = 0.0;
      marker_pub.publish(marker);      
    }
    //change the measured value of the object into an equal distance in rviz 
    if(msg->data >= 5 && msg->data <= 10){
      marker.color.a = 1.0;	
      marker.pose.position.y = -1.1;
      marker_pub.publish(marker);
    }
    if(msg->data > 10 && msg->data <= 15){
      marker.color.a = 1.0;
      marker.pose.position.y = -0.9;
      marker_pub.publish(marker);
    }
    if(msg->data > 15 && msg->data <=20){
      marker.color.a = 1.0;
      marker.pose.position.y = -0.8;
      marker_pub.publish(marker);
    }
    if(msg->data > 20 && msg->data <= 30){
      marker.color.a = 1.0;
      marker.pose.position.y = -0.7;
      marker_pub.publish(marker);
    }    
    if(msg->data > 30){
      marker.color.a = 1.0;
      marker.pose.position.y = -0.6;
      marker_pub.publish(marker);
    }

}

void Listener::spin(){

  
  
  ros::NodeHandle n;  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //subscribe to value of the sensor
  ros::Subscriber sub = n.subscribe("chatter", 1, &Listener::chatterCallback, this);  
  
  ros::spin();
}

void Listener::init(){


      // Set the color -- 
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

// Set the frame ID and timestamp. 
    marker.header.frame_id = "/motor";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.38;
    marker.scale.y = 0.38;
    marker.scale.z = 0.5;

    marker.lifetime = ros::Duration();
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "listener");

   Listener listener;

   listener.init();
   listener.spin();

  return 0;
}
