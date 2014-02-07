#ifndef __THREEMXL_MICVIEW_H
#define __THREEMXL_MICVIEW_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <threemxl/dxlassert.h>
/// Usage example for CDynamixel and CDynamixelROS
class MicView
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle


  public:
    ros::Publisher audio_pub;
    /// Constructor
    MicView() : nh_("~"){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~MicView()
    {

          
      nh_.shutdown();
    }

    void sendMicData();
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_MICVIEW_H */