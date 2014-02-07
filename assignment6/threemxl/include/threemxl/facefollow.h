#ifndef __THREEMXL_FACEFOLLOW_H
#define __THREEMXL_FACEFOLLOW_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <threemxl/dxlassert.h>
#include <people_msgs/PositionMeasurement.h>
/// Usage example for CDynamixel and CDynamixelROS
class DxlROSFaceFollow
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor1_;   ///< Motor interface
    LxSerial serial_port_; ///< Serial port interface
    


  public:
    double prevAngle,minlim,maxlim;
    ros::NodeHandle n;
    ros::Time current_time_, last_time_; 
    /// Constructor
    DxlROSFaceFollow() : nh_("~"), motor1_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSFaceFollow()
    {
      if (motor1_)
        delete motor1_;


      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    void pos1(double pos);
    void posEnd(double pos);
    double present1();
    void msgCallback(const people_msgs::PositionMeasurement::ConstPtr& msg);
    void chatCallback(const std_msgs::String::ConstPtr& msg);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_FACEFOLLOW_H */