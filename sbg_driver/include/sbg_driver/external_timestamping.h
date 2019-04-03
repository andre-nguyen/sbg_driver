/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    03/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <sara_msgs/UIntStamped.h>
#include <sensor_msgs/Imu.h>
#include <cstdint>
#include <functional>
#include <mutex>

namespace sbg {

enum class SyncState {
  synced,
  not_initalized,
  wait_for_sync,
};

/**
 * @brief Timestamp IMU data using an external timestamp, matched by sequence
 * number. Here IMU stamp refers to the timestamp from the IMU itself whereas
 * hardware stamp means timestamp coming from the external timing hardware.
 */
class ExternalTimestamping {
 public:
  typedef std::function<void(const ros::Time &stamp,
                             const ros::Time &original_stamp,
                             const sensor_msgs::ImuPtr imu)> PubImuFcn;

  // Stamp coming in from the imu
  typedef struct {
    uint32_t seq;
    ros::Time imu_stamp;
    ros::Time arrival_stamp;
    sensor_msgs::ImuPtr imu;
  } data_buffer_type;

  // Stamp coming in from the microcontroller
  typedef struct {
    uint32_t seq;
    ros::Time hardware_stamp;
    ros::Time arrival_stamp;
    void reset() {
      seq = 0;
    }
  } hw_stamp_buffer_type;

  explicit ExternalTimestamping(ros::NodeHandle nh);
  void Setup(const PubImuFcn &pub_fcn, const int &fps,
             const double &static_time_offset);

  /**
   * @brief Reset buffers and enable reading in data.
   */
  void Start();

  /**
   * @brief Buffer imu data
   * @param seq IMU sequence number
   * @param imu_stamp Time stamp from SBG imu
   * @param imu IMU data itself
   */
  void BufferImu(const uint32_t &seq, const ros::Time &imu_stamp,
                 sensor_msgs::ImuPtr imu);

  /**
   * @brief Determine the offset between the first received IMU and the one
   * received from the hardware.
   * @param seq
   * @return
   */
  bool SyncSeqOffset(const uint32_t &seq);

  /**
   * @brief Match and incoming IMU packet to a buffered hardware stamp
   * @param seq
   * @param imu_stamp
   * @param imu
   * @return True if a stamp was found and the IMU data was published.
   */
  bool LookupHardwareStamp(const uint32_t &seq, const ros::Time &imu_stamp,
                           sensor_msgs::ImuPtr imu);

  /**
   * @brief Match an incoming hardware stamp to a buffered imu frame
   * @param hw_stamp_seq
   * @param hw_stamp
   * @param arrival_stamp
   * @return
   */
  bool LookupImu(const uint32_t &hw_stamp_seq, ros::Time &hw_stamp,
                 const ros::Time &arrival_stamp);

  void HardwareStampCallback(const sara_msgs::UIntStamped &stamp);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber hw_stamp_sub_;

  int hw_stamp_seq_offset_ = 0;
  double static_time_offset_;
  int frame_rate_;

  PubImuFcn publish_imu_fcn_;
  SyncState state_;
  data_buffer_type imu_buf_;
  hw_stamp_buffer_type hw_stamp_buf_;
  std::mutex mtx_buf_;

  static const std::string kLogPrefix;

};

}  // namespace sbg
