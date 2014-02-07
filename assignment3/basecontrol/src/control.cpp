#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sstream>
using std::cout;
using std::endl;

class Control
{
public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void init();
  void spin();
  ros::NodeHandle nh_;
  int linear_, angular_,linear2_,angular2_,left2_,right2_;
  std_msgs::String s_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};


void Control::init(){
  //haalt de data van beide joysticks en de r1 en r2 op
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_linear2", linear2_, linear2_);
  nh_.param("axis_angular2", angular2_, angular2_);
  nh_.param("left2", left2_, left2_);
  nh_.param("right2", right2_, right2_);
  s_.data = "0 0 0 0 0 0";
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Control::joyCallback, this);
  vel_pub_ = nh_.advertise<std_msgs::String>("ps3", 1);
}

void Control::spin(){

  ros::Rate loop_rate(100);

  while(ros::ok()){
    ros::spinOnce();
    vel_pub_.publish(s_);  
    loop_rate.sleep();
  }

}

//haalt de benodigde waarden uit joy en published het als een string.
void Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  double a1 = joy->axes[angular_];
  double l1 = joy->axes[linear_];
  double a2 = joy->axes[angular2_];
  double l2 = joy->axes[linear2_];
  double back = joy->axes[left2_];
  double gas = joy->axes[right2_];
  std::stringstream ss;
  ss << l1 << " " << a1 << " " << l2 << " " << a2 << " " << back << " " << gas;
  s_.data =  ss.str();
  vel_pub_.publish(s_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  Control control;
  control.init();
  control.spin();
}
