#ifndef __PEOPLEDET_PEOPLEDET_H
#define __PEOPLEDET_PEOPLEDET_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class ImageResize
{
  protected:
   ros::NodeHandle nh_;   ///< ROS node handle
 
    
  public:
    int count;
    ImageResize(): nh_("~"){};
    ~ImageResize()
    {          
	nh_.shutdown();
    }
    
    void spin();
};

#endif /* __PEOPLEDET_PEOPLEDET_H */