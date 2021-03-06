// Copyright 2019 Andre
#include <sbgEComLib.h>
#include <sbg_msgs/ImuIntegral.h>
#include <sbg_msgs/Magnetometer.h>
#include <sbg_msgs/TriggerEvent.h>
#include <sbg_msgs/UtcTime.h>
#include <sensor_msgs/Imu.h>
#include <algorithm>
#include <memory>
#include "sbg_driver/sbg_driver.h"
#include "sbg_driver/sbg_ecom_ros_conversions.h"

#define SBG_HANDLE_ERROR_RET(code) \
  if (code != SbgErrorCode::SBG_NO_ERROR) { return false; }
#define SBG_PUB(ecom, topic, msg_type) \
  pubs_[ecom] = std::make_unique<ros::Publisher>(pnh_.advertise<msg_type> \
      (topic, 50));
#define SBG_CONFIG(ecom, divider) \
  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A, \
                          SBG_ECOM_CLASS_LOG_ECOM_0, ecom, \
                          divider);
#define SBG_CONFIG_PUB(ecom, divider, topic, msg_type) \
  SBG_CONFIG(ecom, divider) \
  SBG_PUB(ecom, topic, msg_type)
// You have to add std::placeholders:: before _1 because Boost stuck their
// place holder in the global namespace
#define REGISTER_ECOM_CALLBACK(id, fcn) \
  ecom_callbacks_[id] = EcomCallback(std::bind(fcn, this, \
      std::placeholders::_1, std::placeholders::_2));

namespace sbg {

SBGDriver::SBGDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
noexcept :
    nh_(nh), pnh_(pnh),
    imu_seq_(0),
    ekf_quat_buf_(kCircularBufSize),
    // @formatter:off
    output_com_config_({
      {SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_MAIN_LOOP},
      {SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DIV_200},
      {SBG_ECOM_LOG_STATUS,   SBG_ECOM_OUTPUT_MODE_DIV_200},
      {SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_MAIN_LOOP},
      {SBG_ECOM_LOG_MAG,      SBG_ECOM_OUTPUT_MODE_DIV_4},
      {SBG_ECOM_LOG_EVENT_A,  SBG_ECOM_OUTPUT_MODE_NEW_DATA}
    })
    // @formatter:on
{
  // Output A
  auto idx = SBG_ECOM_SYNC_OUT_A;
  sbg_sync_out_conf_[idx].duration = 1000000;
  sbg_sync_out_conf_[idx].outputFunction = SBG_ECOM_SYNC_OUT_MODE_DISABLED;
  sbg_sync_out_conf_[idx].polarity = SBG_ECOM_SYNC_OUT_RISING_EDGE;

  // Output B
  idx = SBG_ECOM_SYNC_OUT_B;
  sbg_sync_out_conf_[idx].duration = 1000000;
  sbg_sync_out_conf_[idx].outputFunction = SBG_ECOM_SYNC_OUT_MODE_DISABLED;
  sbg_sync_out_conf_[idx].polarity = SBG_ECOM_SYNC_OUT_RISING_EDGE;

  // @formatter:off
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_IMU_DATA, &SBGDriver::EcomHandleImu);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_UTC_TIME, &SBGDriver::EcomHandleUtc);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_STATUS,   &SBGDriver::EcomHandleStatus);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_EKF_QUAT, &SBGDriver::EcomHandleEKFQuat);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_MAG,      &SBGDriver::EcomHandleMag);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_EVENT_A,  &SBGDriver::EcomHandleEventA);
  // @formatter:on
}

SBGDriver::~SBGDriver() {
  sbgEComClose(&sbg_handle_);
  sbgInterfaceSerialDestroy(&sbg_interface_);
}

// Trampoline C callback, back to C++
SbgErrorCode ReceiveEcomLogC(SbgEComHandle *,
                             SbgEComClass,
                             SbgEComMsgId msg,
                             const SbgBinaryLogData *pLogData,
                             void *pUserArg) {
  auto driver = static_cast<SBGDriver *>(pUserArg);
  if (driver->ReceiveEcomLog(msg, pLogData)) {
    return SBG_NO_ERROR;
  }
  return SBG_ERROR;
}

bool SBGDriver::Init() {
  SbgErrorCode error;

  error = sbgInterfaceSerialCreate(&sbg_interface_, "/dev/ttyUSB0", 921600);

  SBG_HANDLE_ERROR_RET(error);

  error = sbgEComInit(&sbg_handle_, &sbg_interface_);

  SBG_HANDLE_ERROR_RET(error);

  // Disable all the relevant outputs at start but configure their publishers
  // if relevant
  auto disabled = SBG_ECOM_OUTPUT_MODE_DISABLED;
  SBG_CONFIG_PUB(SBG_ECOM_LOG_IMU_DATA, disabled, "imu", sbg_msgs::ImuIntegral);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_UTC_TIME, disabled, "utc", sbg_msgs::UtcTime);
  SBG_CONFIG(SBG_ECOM_LOG_STATUS, disabled);
  SBG_CONFIG(SBG_ECOM_LOG_EKF_QUAT, disabled);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_MAG, disabled, "mag",
                 sbg_msgs::Magnetometer);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_EVENT_A, disabled, "event_a",
                 sbg_msgs::TriggerEvent);

  // Disable the output trigger
  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_A,
                           &sbg_sync_out_conf_[0]);
  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_B,
                           &sbg_sync_out_conf_[1]);

  // Enable input trigger pps
  SbgEComSyncInConf sync_in_a;
  sync_in_a.delay = 0;
  sync_in_a.sensitivity =
      SbgEComSyncInSensitivity::SBG_ECOM_SYNC_IN_RISING_EDGE;
  sbgEComCmdSyncInSetConf(&sbg_handle_, SBG_ECOM_SYNC_IN_A, &sync_in_a);

  // apply and reboot
  error = sbgEComCmdSettingsAction(&sbg_handle_, SBG_ECOM_SAVE_SETTINGS);
  SBG_HANDLE_ERROR_RET(error);

  // Expose service call to enable everything later
  enable_stream_service_ = pnh_.advertiseService("enable",
                                                 &SBGDriver::EnableStreamCallback,
                                                 this);

  error = sbgEComSetReceiveLogCallback(&sbg_handle_, ReceiveEcomLogC, this);
  SBG_HANDLE_ERROR_RET(error);

  bool auto_start = false;
  pnh_.param<bool>("auto_start", auto_start, false);
  if (auto_start) {
    std_srvs::SetBoolRequest req;
    req.data = true;
    std_srvs::SetBoolResponse resp;
    EnableStreamCallback(req, resp);
  }

  return true;
}

bool SBGDriver::EnableStreamCallback(std_srvs::SetBoolRequest &request,
                                     std_srvs::SetBoolResponse &response) {
  if (request.data) {
    for (auto com : output_com_config_) {
      SBG_CONFIG(com.first, com.second);
    }
    sbg_sync_out_conf_[1].outputFunction = SBG_ECOM_SYNC_OUT_MODE_MAIN_LOOP;
    imu_seq_ = 0;
  } else {
    for (auto com : output_com_config_) {
      SBG_CONFIG(com.first, SBG_ECOM_OUTPUT_MODE_DISABLED);
    }
    sbg_sync_out_conf_[1].outputFunction = SBG_ECOM_SYNC_OUT_MODE_DISABLED;
  }

  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_B,
                           &sbg_sync_out_conf_[1]);

  auto error = sbgEComCmdSettingsAction(&sbg_handle_, SBG_ECOM_SAVE_SETTINGS);
  response.success = (error == SBG_NO_ERROR);
  return true;
}

bool SBGDriver::ReceiveEcomLog(SbgEComMsgId msg,
                               const SbgBinaryLogData *data) {
  ros::Time now = ros::Time::now();
  auto it = ecom_callbacks_.find(msg);
  if (it != ecom_callbacks_.end()) {
    it->second(now, data);
  } else {
    ROS_WARN_THROTTLE(10, "Unhandled Ecom log %d", msg);
    return false;
  }

  return true;
}

void SBGDriver::RunOnce() {
  // Handle all buffered logs.
  sbgEComHandle(&sbg_handle_);
}

void SBGDriver::EcomHandleImu(const ros::Time &now,
                              const SbgBinaryLogData *data) {
  imu_buf_.imu = data->imuData;
  imu_buf_.arrival_time = now;
  imu_buf_.orientation_valid = false;
}

void SBGDriver::EcomHandleUtc(const ros::Time &/*now*/,
                              const SbgBinaryLogData *data) {
  pubs_[SBG_ECOM_LOG_UTC_TIME]->publish(UtcToRosUtc(data->utcData));
}

void SBGDriver::EcomHandleStatus(const ros::Time &/*now*/,
                                 const SbgBinaryLogData * /*data*/) {
  // ROS_INFO_THROTTLE(5, "Status %d", data->statusData.timeStamp);
}

void SBGDriver::EcomHandleEKFQuat(const ros::Time &/*now*/,
                                  const SbgBinaryLogData *data) {
  if (data->ekfQuatData.timeStamp == imu_buf_.imu.timeStamp) {
    sbg_msgs::ImuIntegral imu = ImuToImuInt(imu_buf_.imu);
    imu.header.stamp = imu_buf_.arrival_time;
    QuatToRosQuatCov(data->ekfQuatData, &imu.imu.orientation,
                     &imu.imu.orientation_covariance);
    this->pubs_[SBG_ECOM_LOG_IMU_DATA]->publish(imu);
  }
//  ekf_quat_buf_.push_back(data->ekfQuatData);
//  TryPublishImuIntegral();
}

void SBGDriver::EcomHandleMag(const ros::Time &/*now*/,
                              const SbgBinaryLogData *data) {
  pubs_[SBG_ECOM_LOG_MAG]->publish(MagToRosMag(data->magData));
}

void SBGDriver::EcomHandleEventA(const ros::Time &/*now*/,
                                 const SbgBinaryLogData *data) {
  auto evt = EventToRosEvent(data->eventMarker);
  evt.header.stamp = ros::Time::now();
  pubs_[SBG_ECOM_LOG_EVENT_A]->publish(evt);
}

void SBGDriver::TryPublishImuIntegral(bool force_publish) {
  for (auto it = ekf_quat_buf_.begin(); it != ekf_quat_buf_.end(); ++it) {
    if (it->timeStamp == imu_buf_.imu.timeStamp) {
      // Match
      sbg_msgs::ImuIntegral imu = ImuToImuInt(imu_buf_.imu);
      imu.header.frame_id = "imu_link_ned";
      imu.header.stamp = imu_buf_.arrival_time;
      QuatToRosQuatCov(*it, &imu.imu.orientation,
          &imu.imu.orientation_covariance);
      this->pubs_[SBG_ECOM_LOG_IMU_DATA]->publish(imu);

      ekf_quat_buf_.erase(ekf_quat_buf_.begin(), it);
      return;
    }
  }

  // Publish even if orientation not found
  if (force_publish) {
    sbg_msgs::ImuIntegral imu = ImuToImuInt(imu_buf_.imu);
    imu.header.stamp = imu_buf_.arrival_time;
    this->pubs_[SBG_ECOM_LOG_IMU_DATA]->publish(imu);
  }
}

}  // namespace sbg
