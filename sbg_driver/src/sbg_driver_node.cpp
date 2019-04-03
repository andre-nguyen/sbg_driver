// Copyright 2019 Andre
#include <ros/ros.h>
#include "sbg_driver/sbg_driver.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sbg_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sbg::SBGDriver driver(nh, pnh);
  if (!driver.Init()) {
    ROS_FATAL("Unable to initialize.");
    return 1;
  }

  ROS_INFO("SBG IMU Connected.");

  ros::Rate r(250);
  while (ros::ok()) {
    driver.RunOnce();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
