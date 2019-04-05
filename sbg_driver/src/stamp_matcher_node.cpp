/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    05/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#include <ros/ros.h>
#include "sbg_driver/stamp_matcher.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "stamp_matcher_node");

  ros::NodeHandle pnh("~");
  sbg::StampMatcher sm(pnh);

  ros::spin();

  return 0;
}