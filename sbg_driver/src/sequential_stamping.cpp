// Copyright 2019 Andre

#include "sbg_driver/sbg_ecom_ros_conversions.h"
#include "sbg_driver/sequential_stamping.h"

namespace sbg {

SequentialTimestamping::SequentialTimestamping(const ros::NodeHandle &nh) :
    nh_(nh),
    last_imu_sbg_stamp_us_(0),
    last_hw_stamp_seq_(0),
    sequence_(0),
    imu_to_hw_seq_offset_(-1),
    dropped_imu_cnt_(0),
    dropped_hw_stamp_cnt_(0),
    is_started_(false),
    is_first_imu_(true),
    is_first_stamp_(true),
    imu_fifo_(50), stamp_fifo_(50) {
}

bool SequentialTimestamping::Setup() {
  stamp_sub_ = nh_.subscribe("hw_stamp", 100,
                             &SequentialTimestamping::StampCallback, this);
  return true;
}

bool SequentialTimestamping::Start() {
  is_started_ = true;
  is_first_imu_ = true;
  is_first_stamp_ = true;
  sequence_ = 0;
  dropped_imu_cnt_ = 0;
  dropped_hw_stamp_cnt_ = 0;
  return true;
}

bool SequentialTimestamping::AddImu(const ros::Time &arrival_time,
                                    const SbgLogImuData &imu) {
  if (!is_started_) return false;

  if (!is_first_imu_) {
    last_imu_sbg_stamp_us_ = imu.timeStamp;
    is_first_imu_ = false;
  } else {
    const unsigned int k200HzPeriodUs = 5000;
    unsigned int time_diff = imu.timeStamp - last_imu_sbg_stamp_us_;
    unsigned int div = time_diff / k200HzPeriodUs;
    if (div > 1) {
      // Dropped IMU detected, we should drop a hw time stamp
      dropped_imu_cnt_ += div - 1;
      sequence_ += div - 1;
    }

    last_imu_sbg_stamp_us_ = imu.timeStamp;
  }

  ImuBufType ibt;
  ibt.imu_data = imu;
  ibt.arrival_time = arrival_time;
  ibt.sequence = sequence_++;
  imu_fifo_.push_back(ibt);

  TryMatchAndPublishImu();
  return true;
}

bool SequentialTimestamping::AddQuaternion(const SbgLogEkfQuatData &quat) {
  if (!is_started_) return false;

  for (auto it : imu_fifo_) {
    if (it.imu_data.timeStamp == quat.timeStamp) {
      QuatToRosQuatCov(quat,
                       &it.ros_imu.imu.orientation,
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

  if (is_first_stamp_) {
    last_hw_stamp_seq_ = stamp->seq;
    is_first_stamp_ = false;
  } else {
    unsigned int seq_diff = stamp->seq - last_hw_stamp_seq_;
    if (seq_diff > 1) {
      // Dropped hw timestamp detected we should drop an imu message
      dropped_hw_stamp_cnt_ += seq_diff - 1;
    }

    last_hw_stamp_seq_ = stamp->seq;
  }

  ExternalStampType est;
  est.sequence = stamp->seq;
  est.stamp = stamp->stamp;
  stamp_fifo_.push_back(est);

  TryMatchAndPublishStamp();
}

void SequentialTimestamping::TryMatchAndPublishImu() {
  if (stamp_fifo_.empty()) { return; }

  if (imu_to_hw_seq_offset_ < 0) {
    imu_to_hw_seq_offset_ = stamp_fifo_.front().sequence -
        imu_fifo_.front().sequence;
  }

  // Make sure it's the right sequence number
  int seq_offset = stamp_fifo_.front().sequence - imu_fifo_.front().sequence;
  if (seq_offset == imu_to_hw_seq_offset_) {
    // Match!
  } else if (seq_offset > imu_to_hw_seq_offset_) {
    // Can't match this IMU, drop it.

  } else if (seq_offset < imu_to_hw_seq_offset_) {
    // Can't match this stamp, drop it
  }

}

void SequentialTimestamping::TryMatchAndPublishStamp() {
  if (imu_fifo_.empty()) { return; }

  if (imu_to_hw_seq_offset_ < 0) {
    imu_to_hw_seq_offset_ = stamp_fifo_.front().sequence -
        imu_fifo_.front().sequence;
  }

}

}  // namespace sbg
