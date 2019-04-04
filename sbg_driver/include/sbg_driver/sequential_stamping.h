// Copyright 2019 Andre

#ifndef SBG_DRIVER_SEQUENTIAL_STAMPING_H_
#define SBG_DRIVER_SEQUENTIAL_STAMPING_H_

#include <ros/ros.h>
#include <sara_msgs/UIntStamped.h>
#include <sbg_msgs/ImuIntegral.h>
#include <sbgECom.h>
#include <boost/circular_buffer.hpp>
#include <functional>

namespace sbg {

/**
 * This time stamp scheme assumes that the actual hardware can't mess up and
 * the triggers will always get to the microcontroller. However,
 */
class SequentialTimestamping {
 public:
  typedef std::function<void(sbg_msgs::ImuIntegral &imu)> ImuPublishFcn;

  typedef struct {
    ros::Time arrival_time;
    unsigned int sequence;
    SbgLogImuData imu_data;
    bool orientation_valid_ = false;
    sbg_msgs::ImuIntegral ros_imu;
  } ImuBufType;

  typedef struct {
    unsigned int sequence;
    ros::Time stamp;
  } ExternalStampType;

  explicit SequentialTimestamping(const ros::NodeHandle &nh);

  bool Setup();
  bool Start();

  bool AddImu(const ros::Time &arrival_time,
              const SbgLogImuData &imu);

  bool AddQuaternion(const SbgLogEkfQuatData &quat);

  void StampCallback(const sara_msgs::UIntStamped::ConstPtr &stamp);

  void TryMatchAndPublishImu();
  void TryMatchAndPublishStamp();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber stamp_sub_;
  unsigned int last_imu_sbg_stamp_us_;
  unsigned int last_hw_stamp_seq_;
  unsigned int sequence_;
  int imu_to_hw_seq_offset_;           // Always do hw_seq - imu_seq
  unsigned int dropped_imu_cnt_;       // Counter to detect missing imu messages
  unsigned int dropped_hw_stamp_cnt_;  // Counter to detect missing hw stamp
  bool is_started_;
  bool is_first_imu_;
  bool is_first_stamp_;

  boost::circular_buffer<ImuBufType> imu_fifo_;
  boost::circular_buffer<ExternalStampType> stamp_fifo_;
};

}  // namespace sbg

#endif  // SBG_DRIVER_SEQUENTIAL_STAMPING_H_
