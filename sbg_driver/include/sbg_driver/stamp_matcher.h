/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    05/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#pragma once

#include <sara_msgs/UIntStamped.h>
#include <sbg_msgs/ImuIntegral.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>

namespace sbg {

class StampMatcher {
 public:
  StampMatcher(ros::NodeHandle nh);

  void Callback(const sara_msgs::UIntStampedConstPtr &stamp,
                const sbg_msgs::ImuIntegralConstPtr &imu);
 private:
  ros::NodeHandle nh_;

  using UIntStamped = sara_msgs::UIntStamped;
  using ImuIntegral = sbg_msgs::ImuIntegral;
  using ApproximateTimeSync = message_filters::sync_policies::ApproximateTime
      <UIntStamped, ImuIntegral>;
  using Synchronizer = message_filters::Synchronizer<ApproximateTimeSync>;

  message_filters::Subscriber<UIntStamped> stamp_sub_;
  message_filters::Subscriber<ImuIntegral> imu_sub_;
  Synchronizer synchronizer_;
  bool first_;
  bool warned_;
  int imu_to_stamp_offset_;

  ros::Publisher imu_pub_;
  ros::Publisher std_imu_pub_;
};

}  // namespace sbg
