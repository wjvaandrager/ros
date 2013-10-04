#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <phidget_ik/phidget_ik.h>
#include <iostream>
#include "std_msgs/Int32.h"

#include <sstream>
#define SENSORPORT 2
#define OUTPUTPORT 3
PhidgetIK phidget_ik;
//measure how close the object by substracting the refValue from the measured value
int measure(int refValue){
	phidget_ik.setOutputState(OUTPUTPORT, 0);
	usleep(30000);
	return phidget_ik.getSensorValue(SENSORPORT) - refValue;
}
//measure a reference value to compensate light changes
int setRefValue(){
		
	phidget_ik.setOutputState(OUTPUTPORT, 1);
	usleep(30000);//Sleep 30ms
	return phidget_ik.getSensorValue(SENSORPORT);
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "sensor");
	ros::NodeHandle n;


        ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000);

        ros::Rate loop_rate(10000);
	phidget_ik.init(-1); //don't care which phidget
	phidget_ik.waitForAttachment(1000);
	phidget_ik.setRatiometric(1); //Ratriometric on (scaled to input voltage)
	phidget_ik.setDataRate(3,1);
	int NInputs = phidget_ik.getInputCount();
  	int refValue = 0;
  	if (phidget_ik.getLastError()){
    	    std::cerr << "Error initializing PhidgetInterfaceKit: " << phidget_ik.getErrorDescription(phidget_ik.getLastError()) << std::endl;
    	    return 1;
  	}
	while (ros::ok()){
	

	  refValue = setRefValue();
	   
	  std_msgs::Int32 msg;

    	  std::stringstream ss;
    	  ss << measure(refValue);
    	  msg.data = measure(refValue);
          //publish value of object to "chatter" topic
  	  chatter_pub.publish(msg);

    	  ros::spinOnce();

    	  loop_rate.sleep();
	}
	return 0;
}


