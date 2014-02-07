#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/String.h>
#include <sstream>
#include <basecontrol/array.h>

void Array::spin(){
  ros::Rate looprate(5);
  array_pub_ = nh_.advertise<std_msgs::String>("/arduino", 1);
  std_msgs::String s;
  std::stringstream ss;
  int array[] = {0,0,0,0,0};
  for (int i = 0; i < array.size();i++){
    if(i == array.size() - 1){
      ss << array[i];
    }else{
      ss << array[i] << " ";
    }
       
    
  }
  s = ss.str();
  while(ros::ok()){
    array_pub_.publish(s);
    ros::spinOnce();
    looprate.sleep();
  }
  /*
  std::vector<std::string> strs;
  boost::split(strs, msg->data, boost::is_any_of("\t "));
  //haalt de benodigde double uit de message
  double angular = atof( strs[3].c_str() );
  */
}

int main(int argc, char** argv){
  ros::init(argc, argv, "array);
  
  Array a;
  a.spin();
  return 0;
}