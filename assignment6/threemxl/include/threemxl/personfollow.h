#ifndef __THREEMXL_PERSONFOLLOW_H
#define __THREEMXL_PERSONFOLLOW_H

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
class DxlROSPersonFollow
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor1_;
    CDxlGeneric *motor2_; ///< Motor interface
    LxSerial serial_port_; ///< Serial port interface
    


  public:
    ros::NodeHandle n;
    ros::Time current_time_, last_time_;  
    double wheel_diameter_, wheel_base_;
    /// Constructor
    DxlROSPersonFollow() : nh_("~"), motor1_(NULL),motor2_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSPersonFollow()
    {
      if (motor1_)
        delete motor1_;
      if (motor2_)
        delete motor2_;
      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    void pos(double pos,double angle);
    void turn(double angle);
    double linearToRad(double dist);
    double present(int motor);
    void msgCallback(const people_msgs::PositionMeasurement::ConstPtr& msg);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_PERSONFOLLOW_H */
