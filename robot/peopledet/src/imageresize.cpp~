#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <peopledet/imageresize.h>
#include <sstream>
#include <algorithm>
#include <stdlib.h> 

using std::cout;
using std::endl;
using namespace cv;


void ImageResize::spin(){
  count = 0;
  ros::Rate loop_rate(50);
  while(ros::ok()){
    std::stringstream ss;
    ss << "/home/pc_willem/ros/trainHOG/pos/" << count << ".jpg";
    Mat image = imread(ss.str(), 0);
    resize(image, image, Size(64, 128), 0, 0, INTER_CUBIC);
    imwrite(ss.str(),image);
    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imageresize");
  
  ImageResize ir;
  ir.spin();
  return 0;
}