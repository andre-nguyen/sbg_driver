//
// Created by andre on 4/2/19.
//

#ifndef SBG_DRIVER_SBG_DRIVER_H_
#define SBG_DRIVER_SBG_DRIVER_H_

#include <sbgECom.h>
#include <sbgEComLib.h>
#include <ros/ros.h>
#include <sara_msgs/UIntStamped.h>
#include <std_srvs/SetBool.h>
#include <functional>
#include <map>
#include <unordered_map>
#include <string>
#include <tuple>
#include <boost/circular_buffer.hpp>
#include "sbg_driver/external_timestamping.h"

namespace sbg {

class SBGDriver {
 public:
  SBGDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) noexcept;
  ~SBGDriver();
  bool Init();

  /**
   * Service call to enable or disable all outputs
   * @param request
   * @param response
   * @return
   */
  bool EnableStreamCallback(std_srvs::SetBoolRequest &request,
                            std_srvs::SetBoolResponse &response);

  /**
   * Callback when sbgEcom receives a message. We drop the message class
   * argument because for the Ellipse we are only using PORTA. Class would
   * only matter if we decide to use the high speed PortE.
   * @param msg
   * @param data
   */
  bool ReceiveEcomLog(SbgEComMsgId msg, const SbgBinaryLogData *data);

  void RunOnce();

 private:
  typedef struct {
    SbgLogImuData imu;
    ros::Time arrival_time;
    bool orientation_valid = false;
  } ImuIntegralBufType;

  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<ros::Publisher> pubs_[SBG_ECOM_LOG_ECOM_NUM_MESSAGES];
  ros::ServiceServer enable_stream_service_;

  SbgInterface sbg_interface_;
  SbgEComHandle sbg_handle_;
  SbgEComSyncOutConf sbg_sync_out_conf_[2];

  unsigned int imu_seq_;
  ImuIntegralBufType imu_buf_;
  static constexpr size_t kCircularBufSize = 3;
  boost::circular_buffer<SbgLogEkfQuatData> ekf_quat_buf_;

  // ecom log -> output frequency
  std::map<SbgEComLog, SbgEComOutputMode> output_com_config_;

  // Ecom callbacks
  using EcomCallback = std::function<void(const ros::Time &,
                                          const SbgBinaryLogData *)>;
  std::unordered_map<SbgEComMsgId, EcomCallback> ecom_callbacks_;

  // @formatter:off
  void EcomHandleImu    (const ros::Time &, const SbgBinaryLogData *);
  void EcomHandleUtc    (const ros::Time &, const SbgBinaryLogData *);
  void EcomHandleStatus (const ros::Time &, const SbgBinaryLogData *);
  void EcomHandleEKFQuat(const ros::Time &, const SbgBinaryLogData *);
  void EcomHandleMag    (const ros::Time &, const SbgBinaryLogData *);
  void EcomHandleEventA (const ros::Time &, const SbgBinaryLogData *);
  // @formatter:on

  /**
   * @brief Try to match imu message with orientation. Only call this when an
   * orientation message is received. Messages shouldn't come out of order
   * and it looks like the orientation always reads out after the imu message.
   * So my theory is that whenever your orientation message comes out it will
   * match the buffered IMU and we should never have to force publish.
   * @param force_publish
   */
  void TryPublishImuIntegral(bool force_publish = false);
};

}  // namespace sbg

#endif  // SBG_DRIVER_SBG_DRIVER_H_
