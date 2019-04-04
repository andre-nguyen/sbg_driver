// Copyright 2019 Andre

#include "sbg_driver/sbg_ecom_ros_conversions.h"
#include "sbg_driver/sequential_stamping.h"

namespace sbg {

SequentialTimestamping::SequentialTimestamping(const ros::NodeHandle &nh) :
    nh_(nh),
    last_imu_sbg_stamp_us_(0),
    is_started_(false),
    imu_fifo_(50), stamp_fifo_(50) {
}

bool SequentialTimestamping::Setup() {
  stamp_sub_ = nh_.subscribe("hw_stamp", 100,
                             &SequentialTimestamping::StampCallback, this);
  return true;
}

bool SequentialTimestamping::Start() {
  is_started_ = true;
  return true;
}

bool SequentialTimestamping::AddImu(const ros::Time &arrival_time,
                                    const unsigned int &seq,
                                    const SbgLogImuData &imu) {
  ImuBufType ibt;
  ibt.imu_data = imu;
  ibt.arrival_time = arrival_time;
  ibt.sequence = seq;
  imu_fifo_.push_back(ibt);
  return true;
}

bool SequentialTimestamping::AddQuaternion(const SbgLogEkfQuatData &quat) {
  for (auto it : imu_fifo_) {
    if (it.imu_data.timeStamp == quat.timeStamp) {
      QuatToRosQuatCov(quat, &it.ros_imu.imu.orientation,
                       &it.ros_imu.imu.orientation_covariance);
      it.orientation_valid_ = true;
      return true;
    }
  }
  return false;
}

void SequentialTimestamping::StampCallback(
    const sara_msgs::UIntStamped::ConstPtr &stamp) {
  if (!is_started_) return;

  ExternalStampType est;
  est.sequence = stamp->seq;
  est.stamp = stamp->stamp;
  stamp_fifo_.push_back(est);
}

}  // namespace sbg
