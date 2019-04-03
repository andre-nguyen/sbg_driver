// Copyright 2019 Andre
#include <ros/ros.h>
#include "sbg_driver/sbg_driver.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sbg_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sbg::SBGDriver driver(nh, pnh);

  // Need to spin because we have 1 subscriber and 1 service server
  ros::spin();

  return 0;
}
