// Copyright 2019 Andre

#ifndef SBG_DRIVER_SEQUENTIAL_STAMPING_H_
#define SBG_DRIVER_SEQUENTIAL_STAMPING_H_

#include <boost/circular_buffer.hpp>
#include <sbg_msgs/ImuIntegral.h>
#include <sbgECom.h>
#include <ros/ros.h>

namespace sbg {

/**
 * This time stamp scheme assumes that the actual hardware can't mess up and
 * the triggers will always get to the microcontroller. However,
 */
class SequentialTimestamping {
 public:
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
              const unsigned int &seq,
              const SbgLogImuData &imu);

  bool AddStamp(const unsigned int &seq,
                const ros::Time &stamp);



 private:
  ros::NodeHandle nh_;
  unsigned int last_imu_sbg_stamp_us_;
  bool is_started_;

  boost::circular_buffer<ImuBufType> imu_fifo_;
  boost::circular_buffer<ExternalStampType> stamp_fifo_;
};

}  // namespace sbg

#endif  // SBG_DRIVER_SEQUENTIAL_STAMPING_H_
