#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <std_msgs/String.h>
#include <boost/algorithm/string.hpp>
#include <stdlib.h> 
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <cmath>    
using std::cout;
using std::endl;

#define REPLY_SIZE 64
#define TIMEOUT 10000

// This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_node");
    ros::NodeHandle n;
    ros::Publisher ard;
    ros::Publisher left;
    ros::Publisher right;
    ard = n.advertise<std_msgs::String>("arduino", 1);
    left = n.advertise<std_msgs::Float64>("leftWheel", 1);
    right = n.advertise<std_msgs::Float64>("rightWheel", 1);
    std_msgs::String s;
    std_msgs::Float64 lw;
    std_msgs::Float64 rw;
    cereal::CerealPort device;
    char reply[REPLY_SIZE];
    double prev1 = 0;
    double prev2 = 0;
    int count =0;
    double dist1;
    double dist2;
    // Change the next line according to your port name and baud rate
    try{ device.open("/dev/ttyACM0", 57600); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    
    ros::Rate r(30);
    while(ros::ok())
    {
        // Send 'R' over the serial port
        //device.write("R");

        // Get the reply, the last value is the timeout in ms
        try{ device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }
        std::vector<std::string> strs;
        boost::split(strs, reply, boost::is_any_of("\t "));
	//checkt of string goed ontvangen is
	if(strs[0] == "0" && strs[3] == "0"){
	  dist1 = atof(strs[1].c_str());
	  dist2 = atof(strs[2].c_str());
	  if (dist1>=300 || dist1 < -1){
	    dist1 =-1;
	  }if (dist2>=300 || dist2 < -1){
	    dist2 =-1;
	  }
	  //filter kleine pieken eruit
	  if(fabs(dist1 - prev1) < 9){
	    prev1 = dist1;
	    count = 3;
	  }
	  if(fabs(dist2 - prev2) < 9){
	    prev2 = dist2;
	    count = 3;
	  }else{
	    count++;
	  }
	}
	//publish na derde nieuwe waarde of als de waarde dicht genoeg bij vorige zit
	if(count >=3){
	  std::stringstream ss;
	  ss << dist2 << " " << dist1;
	  s.data =  ss.str();
	  rw.data = dist1;
	  lw.data = dist2;
	  ard.publish(s);
	  left.publish(lw);
	  right.publish(rw);
	  prev1 = dist1;
	  prev2 = dist2;
	  count = 0; 
	}
        ros::spinOnce();
        r.sleep();
    }   
}
