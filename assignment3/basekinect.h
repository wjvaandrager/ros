#ifndef __THREEMXL_BASEKINECT_H
#define __THREEMXL_BASEKINECT_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlROSBaseKinect
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
    ros::Publisher angle_pub;
    ros::Publisher pos_pub;
    ros::Publisher dist_pub;
    
    /// Constructor
    DxlROSBaseKinect() : nh_("~"), motor1_(NULL), motor2_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSBaseKinect()
    {
      if (motor1_)
        delete motor1_;

      if (motor2_)
        delete motor2_;

      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }
    void turn(double angle);
    void pos(double pos,double angle);
    void kinectCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    double angleBase(double l1,double l2,double angle);
    double cosRuleLength(double l1,double l2,double angle);
    double cosRuleAngle(double l1,double l2,double l3);
    double distBase(double l1,double l2,double angle);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    void publishStates();
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_BASEKINECT_H */
