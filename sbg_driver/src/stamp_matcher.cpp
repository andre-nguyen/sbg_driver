/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    05/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#include "sbg_driver/stamp_matcher.h"

namespace sbg {

StampMatcher::StampMatcher(ros::NodeHandle nh) :
    nh_(nh),
    stamp_sub_(nh, "stamp", 1),
    imu_sub_(nh, "imu", 1),
    synchronizer_(ApproximateTimeSync(10), stamp_sub_, imu_sub_),
    first_(true),
    warned_(false),
    imu_to_stamp_offset_(0) {
  synchronizer_.registerCallback(boost::bind(&StampMatcher::Callback,
      this, _1, _2));
  imu_pub_ = nh.advertise<sbg_msgs::ImuIntegral>("imu_corrected", 10);
}

void StampMatcher::Callback(const sara_msgs::UIntStampedConstPtr &stamp,
                            const sbg_msgs::ImuIntegralConstPtr &imu) {
  if (first_) {
    imu_to_stamp_offset_ = stamp->seq - imu->header.seq;
    first_ = false;
  }

  if (warned_ &&
      static_cast<int>(stamp->seq - imu->header.seq) != imu_to_stamp_offset_) {
    ROS_WARN_ONCE("Sequence offset changed. This will probably break now. "
                  "This message will only be printed once.");
    warned_ = true;
  }

  sbg_msgs::ImuIntegralPtr restamped(new sbg_msgs::ImuIntegral(*imu));
  restamped->header.stamp = stamp->stamp;
  imu_pub_.publish(restamped);
}

}  // namespace sbg

