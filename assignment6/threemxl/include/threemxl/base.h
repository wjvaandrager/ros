#ifndef __THREEMXL_BASE_H
#define __THREEMXL_BASE_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <threemxl/dxlassert.h>
/// Usage example for CDynamixel and CDynamixelROS
class DxlROSBase
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor1_;   ///< Motor interface
    CDxlGeneric *motor2_;   ///< Motor interface
    LxSerial serial_port_; ///< Serial port interface
    


  public:
    double x_, y_, th_, vx_, vth_, wheel_diameter_, wheel_base_,prevLeft_,prevRight_;
    ros::NodeHandle n;
    ros::Publisher speed1_pub;
    ros::Publisher speed2_pub;
    ros::Publisher pos1_pub;
    ros::Publisher pos2_pub;
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Publisher odom_pub_;
    ros::Publisher bl_pub_;
    sensor_msgs::JointState js;
    ros::Publisher joint_pub;
    ros::Time current_time_, last_time_; 
    /// Constructor
    DxlROSBase() : nh_("~"), motor1_(NULL), motor2_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSBase()
    {
      if (motor1_)
        delete motor1_;

      if (motor2_)
        delete motor2_;

      if (serial_port_.is_port_open())
        serial_port_.port_close();
          
      nh_.shutdown();
    }

    void cirkle(double radius,double rotate);
    void cirkle1(double radius,double cirkles);
    void turn(double angle);
    void pos(double pos,double angle);
    void drive(double v,double w);
    double min(double x,double y);
    void corner(double x,double y);
    void setPos(double pos,double wheel);
    void publishOdometry();
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    void publishStates();
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_BASE_H */
