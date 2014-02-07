#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <algorithm> 
#include <sstream>
using namespace std;

class Control
{
public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void init();
  void spin();
  ros::NodeHandle nh_;
  int linear_, angular_,linear2_,angular2_,left2_,right2_;
  std_msgs::String s_;
  ros::Publisher arduinoComb_pub;
  ros::Publisher arduinoLed_pub;
  ros::Publisher head_pub;
  ros::Subscriber joy_sub_;
  int modus;
  bool siren;
  bool strobe;
};


void Control::init(){
  //haalt de data van beide joysticks en de r1 en r2 op
  modus = 0;
  siren = false;
  strobe = false;
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Control::joyCallback, this);
  arduinoComb_pub = nh_.advertise<std_msgs::String>("/arduino/comb", 1);
  arduinoLed_pub = nh_.advertise<std_msgs::String>("/arduino/led", 1);
  head_pub = nh_.advertise<std_msgs::String>("/chat", 1);
}

void Control::spin(){

  ros::Rate loop_rate(30);

  while(ros::ok()){
    ros::spinOnce(); 
    loop_rate.sleep();
  }

}

//haalt de benodigde waarden uit joy en published het als een string.
void Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  double tr = joy->axes[12];
  double cir = joy->axes[13];
  double x = joy->axes[14];
  double sq = joy->axes[15];
  double up = joy->axes[4];
  double right = joy->axes[5];
  double down = joy->axes[6];
  double left = joy->axes[7];
  double reset = joy->axes[11];
//   double back = joy->axes[8];
//   double gas = joy->axes[9];
  std_msgs::String head;
  if(sq<-0.5){
    head.data = "pos 5";
    head_pub.publish(head);
  }else if (x < -0.5){
    head.data = "pos -5";
    head_pub.publish(head);
  }
  if(cir<-0.8 || tr <-0.8){
    
    stringstream ac;
    if(cir<-0.8){
      if(strobe){
	ac << "strobe_off iets ";
	strobe = false;
      }else{
        ac << "strobe_on iets ";
	strobe = true;
      }
    }
    if(tr<-0.8){
      if(siren){
        ac << "sirenSoft_off iets ";
	siren = false;
      }else{
	ac << "sirenSoft_on iets ";
	siren = true;
      }
    }
    s_.data = ac.str();
    arduinoComb_pub.publish(s_);
  }
  std_msgs::String sal;
  if(up < -0.8){
    sal.data = "1";
    arduinoLed_pub.publish(sal);
  }else if(right < -0.8){
    sal.data = "2";
    arduinoLed_pub.publish(sal);
  }else if(down < -0.8){
    sal.data = "3";
    arduinoLed_pub.publish(sal);
  }else if(left < -0.8){
    sal.data = "4";
    arduinoLed_pub.publish(sal);
  }else if(reset < -0.8){
    sal.data = "0";
    arduinoLed_pub.publish(sal);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  Control control;
  control.init();
  control.spin();
}
