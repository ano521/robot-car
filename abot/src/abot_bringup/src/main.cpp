#include <ros/ros.h>
#include "abot_bringup/base_driver.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "abot_driver");

  abot::BaseDriver::Instance()->work_loop();

  ros::spin();


  return 0;
}

