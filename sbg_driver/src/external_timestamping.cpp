/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    03/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#include "sbg_driver/external_timestamping.h"

namespace sbg {

const std::string ExternalTimestamping::kLogPrefix = "[HW timesync] ";

ExternalTimestamping::ExternalTimestamping(ros::NodeHandle nh) :
    nh_(nh),
    state_(SyncState::not_initalized) {
  imu_buf_.imu.reset();
  hw_stamp_buf_.reset();
}

void ExternalTimestamping::Setup(const PubImuFcn &pub_fcn, const int &fps,
                                 const double &static_time_offset) {
  static_time_offset_ = static_time_offset;
  frame_rate_ = fps;
  publish_imu_fcn_ = pub_fcn;
  state_ = SyncState::not_initalized;
  hw_stamp_sub_ = nh_.subscribe("hw_stamp",
                                100,
                                &ExternalTimestamping::HardwareStampCallback,
                                this);
}

void ExternalTimestamping::Start() {
  if (!publish_imu_fcn_) {
    ROS_ERROR_STREAM(kLogPrefix << "No publish function set, dropping frames.");
  }

  hw_stamp_buf_.reset();
  imu_buf_.imu.reset();

  state_ = SyncState::wait_for_sync;
}

void ExternalTimestamping::BufferImu(const uint32_t &seq,
                                     const ros::Time &imu_stamp,
                                     sensor_msgs::ImuPtr imu) {
  std::lock_guard<std::mutex> lock_guard(mtx_buf_);

  if (imu_buf_.imu) {
    ROS_WARN_STREAM_THROTTLE(1, kLogPrefix << "Overwriting imu buffer.");
  }

  // Set frame buffer
  imu_buf_.imu = imu;
  imu_buf_.seq = seq;
  imu_buf_.arrival_stamp = ros::Time::now();
  imu_buf_.imu_stamp = imu_stamp;
}

bool ExternalTimestamping::SyncSeqOffset(const uint32_t &seq) {
  hw_stamp_seq_offset_ = hw_stamp_buf_.seq - static_cast<int32_t>(seq);

  ROS_INFO_THROTTLE(1,
                    "%s New seq offset: %d, from %d to %d",
                    kLogPrefix.c_str(),
                    hw_stamp_seq_offset_,
                    hw_stamp_buf_.seq,
                    seq);

  state_ = SyncState::synced;
  return true;
}

bool ExternalTimestamping::LookupHardwareStamp(const uint32_t &seq,
                                               const ros::Time &imu_stamp,
                                               sensor_msgs::ImuPtr imu) {
  std::lock_guard<std::mutex> lock_guard(mtx_buf_);

  if (state_ == SyncState::not_initalized) {
    ROS_WARN_THROTTLE(1, "%s Sync state not init.", kLogPrefix.c_str());
    return false;
  }

  if (hw_stamp_buf_.seq == 0) {
    ROS_WARN_THROTTLE(1, "%s Stamp buf empty.", kLogPrefix.c_str());
    return false;
  }

  // set max allowed age of buffered stamp to the interval between two frames
  const double kMaxStampAge = 1.0 / frame_rate_;
  const double age_buffered_hw_stamp = imu_stamp.toSec() -
      hw_stamp_buf_.arrival_stamp.toSec();
  if (std::fabs(age_buffered_hw_stamp) > kMaxStampAge) {
    ROS_WARN_THROTTLE(1, "%s Delay out of bounds: %f s vs %f. Clearing "
                          "buffer", kLogPrefix.c_str(),
                       kMaxStampAge, age_buffered_hw_stamp);
    hw_stamp_buf_.reset();
    state_ = SyncState::wait_for_sync;
    return false;
  }

  uint32_t expected_hw_stamp_seq = seq + hw_stamp_seq_offset_;
  if (state_ == SyncState::wait_for_sync ||
      hw_stamp_buf_.seq != expected_hw_stamp_seq) {
    // there is a buffered stamp and it is within the expected time interval
    // however, we did not determine the sequence yet or the seq id did not
    // match the one we expected call syncSeqOffset to set or reset the seq id
    // offset between frames and stamps
    ROS_WARN("%s Dropped stammp: could not find hw stamp with seq "
             "id: %d", kLogPrefix.c_str(), expected_hw_stamp_seq);
    SyncSeqOffset(seq);
  }

  // successful match: publish frame
  publish_imu_fcn_(hw_stamp_buf_.hardware_stamp, imu_buf_.imu_stamp, imu);
  ROS_INFO_THROTTLE(5,
                    "%s frame#: %d stamp#: %d t_imu: %f t_hw: %f delay: %f",
                    kLogPrefix.c_str(),
                    seq,
                    expected_hw_stamp_seq,
                    imu_stamp.toSec(),
                    hw_stamp_buf_.hardware_stamp.toSec(),
                    age_buffered_hw_stamp);

  return true;
}

bool ExternalTimestamping::LookupImu(const uint32_t &hw_stamp_seq,
                                     ros::Time &hw_stamp,
                                     const ros::Time &arrival_stamp) {
  std::lock_guard<std::mutex> lock_guard(mtx_buf_);

  if (!imu_buf_.imu) {
    return false;
  }

  // currently set to 0 as we expect hw stamps to arrive before the frame
  const double kMaxFrameAge = 0e-3;
  const double age_buffered_frame = arrival_stamp.toSec() -
      imu_buf_.arrival_stamp.toSec();
  if (std::fabs(age_buffered_frame) > kMaxFrameAge) {
    ROS_WARN_THROTTLE(1, "%s Delay out of bounds %f s. Dropping "
                          "frame", kLogPrefix.c_str(),
                       age_buffered_frame);
    imu_buf_.imu.reset();
    state_ = SyncState::wait_for_sync;
    return false;
  }

  uint32_t expected_frame_seq = hw_stamp_seq - hw_stamp_seq_offset_;
  if (state_ == SyncState::wait_for_sync ||
      imu_buf_.seq != expected_frame_seq) {
    SyncSeqOffset(hw_stamp_seq);
    ROS_WARN("%s Dropped frame: coundl't find frame with seq %d.",
             kLogPrefix.c_str(), expected_frame_seq);
  }

  publish_imu_fcn_(hw_stamp, imu_buf_.imu_stamp, imu_buf_.imu);
  ROS_INFO_THROTTLE(5,
                    "%s frame#: %d stamp#: %d t_imu: %f t_hw: %f delay: %f",
                    kLogPrefix.c_str(),
                    expected_frame_seq,
                    hw_stamp_seq,
                    imu_buf_.imu_stamp.toSec(),
                    hw_stamp.toSec(),
                    age_buffered_frame);
  return true;
}

void ExternalTimestamping::HardwareStampCallback(
    const sara_msgs::UIntStamped &stamp) {
  if (state_ == SyncState::not_initalized) {
    ROS_ERROR_THROTTLE(1, "fuck");
    return;
  }

  ROS_ERROR_THROTTLE(1, "should go through");

  ros::Time arrival_stamp = ros::Time::now();
  ros::Time hw_stamp = stamp.stamp;
  uint32_t hw_stamp_seq = stamp.seq;

  if (!LookupImu(hw_stamp_seq, hw_stamp, arrival_stamp)) {
    imu_buf_.imu.reset();
    hw_stamp_buf_.seq = hw_stamp_seq;
    hw_stamp_buf_.arrival_stamp = arrival_stamp;
    hw_stamp_buf_.hardware_stamp = hw_stamp;
    ROS_ERROR_THROTTLE(1, "set");
  }
}

}  // namespace sbg
