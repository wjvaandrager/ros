#ifndef __THREEMXL_BASEARDUINO_H
#define __THREEMXL_BASEARDUINO_H

#include <ros/ros.h>
#include <CDxlGeneric.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlROSBaseArduino
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
    ros::Publisher pos1_pub;
    ros::Publisher pos2_pub;
    
    /// Constructor
    DxlROSBaseArduino() : nh_("~"), motor1_(NULL), motor2_(NULL){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSBaseArduino()
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
    void arduinoCallback(const std_msgs::String::ConstPtr& msg);
    /// Initialize node
    /** \param path path to shared_serial node */
    void init(char *path);
    void publishStates();
    /// Spin
    void spin();
};    

#endif /* __THREEMXL_BASEARDUINO_H */
