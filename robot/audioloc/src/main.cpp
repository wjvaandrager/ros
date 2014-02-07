#include <iostream>
#include "audioloc/fasttransforms.h" 
#include "audioloc/alglibmisc.h"
#include "audioloc/alglibinternal.h"
#include "audioloc/main.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <audioloc/MultiArrayFloat32.h>
#include <math.h>  
#include <algorithm>  
#include <sstream>
#include <ros/ros.h>
#define SAMPLESPACE 8005
#define SAMPLERATE 15360.0
#define numberOfPeaks 1 //shut up
#define speedOfSound 341.0
#define RAD (M_PI/180)
using namespace alglib;
using namespace std;
float micPositions[4] = {-113, 36, 76, 113};
float lastAngle = 0;

double findAngle(double *d,int *m){ //if peakposition > 10, out of bounds due to speed of sound and sample rate 
  int peakPosition = 0;
  for(int i =0; i<SAMPLESPACE*2-1; i++){  
    //cout << d[i] << " ";
    if(d[i]>d[peakPosition]){
      peakPosition = i; 
    }
    if(i == 12){
      i = SAMPLESPACE*2 - 13;
    }
  }
  if(peakPosition > SAMPLESPACE){
    peakPosition -= SAMPLESPACE*2-1; 
  }
  if(fabs(peakPosition)<=11 ){
    double val = peakPosition/SAMPLERATE/(((micPositions[m[0]]-micPositions[m[1]])/1000)/speedOfSound); 
    if(fabs(val)<=1){
      return asin(val);
    }else{
      return -1000;
    }
  }
  return -1000;
}

real_1d_array xcorr(double *data1, double *data2){ // do the cross correlation
  real_1d_array signal;
  real_1d_array pattern;
  real_1d_array corrResult;
  
  signal.setcontent(SAMPLESPACE, data1);
  pattern.setcontent(SAMPLESPACE, data2);
  
  corrr1d(signal, SAMPLESPACE, pattern, SAMPLESPACE, corrResult);
  
  return corrResult;
}

double disatance(double alpha, double beta, double a, double b){ // y distance
  return (a-b)*sin(alpha)*sin(-beta)/sin(alpha-beta);
}

double horizontal(double beta, double distance, double b){ // x distance 
  return (distance/tan(-beta)+b);
}

void Sound::calculateLoc(const audioloc::MultiArrayFloat32::ConstPtr& msg){
  double data[4][msg->mic1.size()];
  copy(msg->mic1.begin(),msg->mic1.end(), data[0] );
  copy(msg->mic2.begin(),msg->mic2.end(), data[1] );
  copy(msg->mic3.begin(),msg->mic3.end(), data[2] );
  copy(msg->mic4.begin(),msg->mic4.end(), data[3] );
  double maxValue=0;
  for(int i = 0; i< SAMPLERATE; i++){
    if(fabs(data[0][i])>maxValue)
      maxValue=fabs(data[0][i]);
  }
  if(maxValue > 20000000){
    double *d;
    int m[2] = {0,0};
    double theta[2][6]; // array of angles at positions 
    int k = 0;
    for(int i = 0; i<3; i++){
      for(int j = i+1; j<4; j++ ){
	m[0] = i;
	m[1] = j;
	if(i!=j){
	  d = xcorr(data[i], data[j]).getcontent();
	  theta[0][k] = findAngle(d,m);
	  theta[1][k] = (micPositions[i] + micPositions[j])/2;
	  k++;
	}
      }
    }
    double average = 0;
    double count = 0;
    for(int i =0; i< 3;i++){
      if(theta[0][i] != -1000){
	//cout << i << ": " << theta[0][i] << " ";
	average += theta[0][i];
	count++;
      }
    }
    if(count >= 3){
      //cout << endl;
      average/=count;
      //cout << average/RAD << endl;
    }
  //   cout << pos[0] << " " << pos[1] << endl;
    //check of vorige hoek in de buurt van hoek is om uitschieters te voorkomen
    if(fabs(average - lastAngle) < 0.1){
      std_msgs::String a;
      stringstream ss;
      ss << average << " " << maxValue;
      a.data = ss.str();
      sound_pub.publish(a);
    }
    lastAngle = average;
  }
  
}

void Sound::state(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data){
    audio_sub = nh_.subscribe("/audio", 1, &Sound::calculateLoc, this);
  }else{
    audio_sub = nh_.subscribe("/audio/fout/dus/dan/krijgt/ie/geen/data/meer/door/topic", 1, &Sound::calculateLoc, this);
  }
}

void Sound::spin(){
  ros::Rate looprate(2);
  audio_sub = nh_.subscribe("/audio", 1, &Sound::calculateLoc, this);
  pause_sub = nh_.subscribe("/visualinput",1, &Sound::state,this);
  sound_pub = nh_.advertise<std_msgs::String>("/sound/localisation", 1);
  while(ros::ok()){
    ros::spinOnce();
    looprate.sleep();
  }

  
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "audio_localisation");
  
  Sound s;
  s.spin();
  return 0;
}


