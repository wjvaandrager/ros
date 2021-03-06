#include <ros/ros.h>
#include <threemxl/example.h>
#include <threemxl/C3mxlROS.h>

#define NUM_SAMPLES  10000
#define NUM_MESSAGES 1
#define NUM_BINS     64
#define NUM_STARS    16

void DxlROSExample::init(char *path)
{
  CDxlConfig *config = new CDxlConfig();

  ROS_INFO("Using direct connection");

  config->mDxlTypeStr = "3MXL";
  motors_ = new CDxlGroup();
  motors_->addNewDynamixel(config->setID(100));
  motors_->addNewDynamixel(config->setID(101));
  motors_->addNewDynamixel(config->setID(102));
  motors_->addNewDynamixel(config->setID(103));

  serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port_.set_speed(LxSerial::S921600);
  motors_->setSerialPort(&serial_port_);

  ROS_ASSERT(motors_->init());
  
  ROS_ASSERT(motors_->setupSyncReadChain() == DXL_SUCCESS);
  ROS_ASSERT(motors_->getDynamixel(0)->set3MxlMode(TORQUE_MODE) == DXL_SUCCESS);
  ROS_ASSERT(motors_->getDynamixel(1)->set3MxlMode(TORQUE_MODE) == DXL_SUCCESS);
  ROS_ASSERT(motors_->getDynamixel(2)->set3MxlMode(TORQUE_MODE) == DXL_SUCCESS);
  ROS_ASSERT(motors_->getDynamixel(3)->set3MxlMode(TORQUE_MODE) == DXL_SUCCESS);

  delete config;
}

void DxlROSExample::spin()
{
  double samples[NUM_SAMPLES];
  double mi=1, ma=0, avg=0;

  printf("Benchmarking");

  // Warm up
  motors_->getStateAll();
  
  for (int ii=0; ii < NUM_SAMPLES; ++ii)
  {
    ros::Time begin = ros::Time::now();
    for (int jj=0; jj < NUM_MESSAGES; ++jj)
      motors_->getStateAll();
    samples[ii] = (ros::Time::now() - begin).toSec();
    mi = fmin(mi, samples[ii]);
    ma = fmax(ma, samples[ii]);
    avg += samples[ii];
    
    if (!(ii % 100))
    {
      printf(".");
      fflush(stdout);
    }
  }
  printf("\n");
  
  avg /= NUM_SAMPLES;
  
  double d = (ma-mi)/(NUM_BINS-1), std=0;
  int histogram[NUM_BINS];
  
  bzero(histogram, NUM_BINS*sizeof(int));
  
  for (int ii=0; ii < NUM_SAMPLES; ++ii)
  {
    int bin = std::min((int)((samples[ii]-mi)/d), NUM_BINS-1);
    histogram[bin]++;
    std += pow((samples[ii]-avg), 2);
  }
  
  std = sqrt(std/NUM_SAMPLES);
  
  printf("3mxl lateny report for %d messages\n", NUM_MESSAGES);
  printf("----------------------------------\n");
  printf("Average           : %f\n", avg);
  printf("Standard deviation: %f\n", std);
  printf("Minimum           : %f\n", mi);
  printf("Maximum           : %f\n", ma);
  printf("\nHistogram\n");
  
  int hmi=NUM_SAMPLES, hma=0;
  for (int ii=0; ii < NUM_BINS; ++ii)
  {
    hmi = std::min(hmi, histogram[ii]);
    hma = std::max(hma, histogram[ii]);
  }
  
  int hd = ceil((hma-hmi)/(NUM_STARS-1));
  
  for (int ii=NUM_STARS-1; ii >= 0; --ii)
  {
    for (int jj=0; jj < NUM_BINS; ++jj)
      if (histogram[jj] >= hmi+ii*hd)
        printf("*");
      else
        printf(" ");
    printf("\n");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_example");

  char *path=NULL;
  if (argc == 2)
    path = argv[1];
 
  DxlROSExample dxl_ros_example;
  
  dxl_ros_example.init(path);
  dxl_ros_example.spin();
  
  return 0;   
} 
