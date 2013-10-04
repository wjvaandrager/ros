#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <visualization_msgs/Marker.h>

/// Usage example for CDynamixel and CDynamixelROS
class Listener
{

  protected:
    ros::NodeHandle nh_;   ///< ROS node handle

  public:
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;



    /// Constructor
    Listener() : nh_("~"){ }
    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~Listener()
    {        
      nh_.shutdown();
    }

    void chatterCallback(const std_msgs::Int32::ConstPtr& m);
    
    void init();
    /// Spin
    void spin();
};  
