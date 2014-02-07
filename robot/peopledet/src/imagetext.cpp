#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <peopledet/imagetext.h>
#include <sstream>
#include <string.h>
#include <algorithm>
#include <stdlib.h> 

using std::cout;
using std::endl;
using namespace cv;


void ImageResize::spin(){
  count = 0;
  string text = "hoi dit is een test";
  string letter = "";
  Mat sentence(8,8*text.size(),CV_8U);
  Mat image;
  imwrite("/home/pc_willem/ros/kinectdata/out.png",sentence);
  for(int k = 0; k < text.size();k++){
    if(text[k] == ' '){
      letter = "space";
    }else{
      letter = text[k];   
    }
   
    std::stringstream ss;
    ss << "/home/pc_willem/ros/peopledet/letters/" << letter << ".png";
     
        
    image = imread(ss.str(), CV_8U);
    for(int i = 0; i < 8; i++){
      
      for(int j = 0; j < image.size().width; j++){
       int num = j+count;
       sentence.at<int>(i,num) = image.at<int>(i,j);
      }
    }
    cout << " " << endl;
//   resize(image, image, Size(64, 128), 0, 0, INTER_CUBIC);
//   imwrite(ss.str(),image);
    count+=image.size().width;
  }
  imwrite("/home/pc_willem/ros/kinectdata/realout.png",sentence);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imagetext");
  
  ImageResize ir;
  ir.spin();
  return 0;
}