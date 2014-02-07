#ifndef __THREEMXL_BASEKINECT1_H
#define __THREEMXL_BASEKINECT1_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlROSBaseKinect1
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor1_;   ///< Motor interface
    CDxlGeneric *motor2_;   ///< Motor interface
    LxSerial serial_port_; ///< Serial port interface
     
  public:
    
    ros::NodeHandle n;
    ros::Publisher speed1_pub;
    ros::Publisher speed2_pub;
    ros::Publisher dist_pub;
    sensor_msgs::LaserScan ls;
    double distance,dis;
    /// Constructor
    DxlROSBaseKinect1() : nh_("~"), motor1_(NULL), motor2_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSBaseKinect1()
    {
      if (motor1_)
        delete motor1_;

      if (motor2_)
        delete motor2_;

      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }
    void ps3Callback(const std_msgs::String::ConstPtr& msg);
    void kinectCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void steer(double linear,double linear2);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(double dist);
    void publishStates();
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_BASEKINECT1_H */
