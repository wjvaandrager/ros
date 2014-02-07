#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <peopledet/ppldet.h>
#include <sstream>
#include <algorithm>
#include <stdlib.h> 

using std::cout;
using std::endl;
using namespace cv;

//converts bag messages to images
void ImageConverter::pplDet(const sensor_msgs::Image::ConstPtr& msg){
  
    cv_bridge::CvImageConstPtr cvImg= cv_bridge::toCvShare(msg);
    //namedWindow("opencv", CV_WINDOW_AUTOSIZE);
    Mat img = cvImg->image;
    img = 50*img;
    img.assignTo(img,CV_8U);
    std::stringstream ss;
    ss << "/home/pc_willem/ros/trainHOG/pos/" << count << ".jpg";
    cout << ss.str() << endl;
    string s =  ss.str();
    imwrite(s,img);
    count++;
    //image.data = img;
//     image_pub_.publish(msg);
}

void ImageConverter::spin(){
  count = 0;
  ros::Rate loop_rate(50);
  depth_sub_ = nh_.subscribe("/camera/depth/test", 1, &ImageConverter::pplDet, this);
  //image_sub_ = nh_.subscribe("/camera/rgb/image_color", 5, &ImageConverter::pplDet, this);
  
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_to_image");
  
  ImageConverter ic;
  ic.spin();
  return 0;
}