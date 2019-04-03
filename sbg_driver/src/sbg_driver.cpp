// Copyright 2019 Andre
#include <sbgEComLib.h>
#include <sbg_msgs/ImuIntegral.h>
#include <sbg_msgs/TriggerEvent.h>
#include <sbg_msgs/UtcTime.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <memory>
#include "sbg_driver/sbg_driver.h"

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
      std::placeholders::_1));

namespace sbg {

SBGDriver::SBGDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
noexcept :
    nh_(nh), pnh_(pnh),
    // @formatter:off
    output_com_config_({
      {SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_MAIN_LOOP},
      {SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DIV_200},
      {SBG_ECOM_LOG_STATUS,   SBG_ECOM_OUTPUT_MODE_DIV_200},
      {SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_MAIN_LOOP},
      {SBG_ECOM_LOG_MAG,      SBG_ECOM_OUTPUT_MODE_DIV_4},
      {SBG_ECOM_LOG_EVENT_A,  SBG_ECOM_OUTPUT_MODE_EVENT_IN_A}
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

  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_IMU_DATA, &SBGDriver::EcomHandleImu);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_UTC_TIME, &SBGDriver::EcomHandleUtc);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_STATUS, &SBGDriver::EcomHandleStatus);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_EKF_QUAT, &SBGDriver::EcomHandleEKFQuat);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_MAG, &SBGDriver::EcomHandleMag);
  REGISTER_ECOM_CALLBACK(SBG_ECOM_LOG_EVENT_A, &SBGDriver::EcomHandleEventA);
}

SBGDriver::~SBGDriver() {
  sbgEComClose(&sbg_handle_);
  sbgInterfaceSerialDestroy(&sbg_interface_);
}

// Trampoline C callback, back to C++
SbgErrorCode ReceiveEcomLogC(SbgEComHandle *pHandle,
                             SbgEComClass msgClass,
                             SbgEComMsgId msg,
                             const SbgBinaryLogData *pLogData,
                             void *pUserArg) {
  auto driver = static_cast<SBGDriver *>(pUserArg);
  driver->ReceiveEcomLog(msgClass, msg, pLogData);
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
  SBG_CONFIG_PUB(SBG_ECOM_LOG_IMU_DATA, disabled, "imu", sensor_msgs::Imu);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_UTC_TIME, disabled, "utc", sbg_msgs::UtcTime);
  SBG_CONFIG(SBG_ECOM_LOG_STATUS, disabled);
  SBG_CONFIG(SBG_ECOM_LOG_EKF_QUAT, disabled);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_MAG, disabled, "mag",
                 sensor_msgs::MagneticField);
  SBG_CONFIG_PUB(SBG_ECOM_LOG_EVENT_A, disabled, "event_a",
                 sbg_msgs::TriggerEvent);

  // Disable the output trigger
  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_A,
                           &sbg_sync_out_conf_[0]);
  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_B,
                           &sbg_sync_out_conf_[1]);

  // apply and reboot
  error = sbgEComCmdSettingsAction(&sbg_handle_, SBG_ECOM_SAVE_SETTINGS);
  SBG_HANDLE_ERROR_RET(error);

  // Expose service call to enable everything later
  enable_stream_service_ = pnh_.advertiseService("enable",
                                                 &SBGDriver::EnableStreamCallback,
                                                 this);

  error = sbgEComSetReceiveLogCallback(&sbg_handle_, ReceiveEcomLogC, this);
  SBG_HANDLE_ERROR_RET(error);

  return true;
}

bool SBGDriver::EnableStreamCallback(std_srvs::SetBoolRequest &request,
                                     std_srvs::SetBoolResponse &response) {
  if (request.data) {
    for (auto com : output_com_config_) {
      SBG_CONFIG(com.first, com.second);
    }
    sbg_sync_out_conf_[1].outputFunction = SBG_ECOM_SYNC_OUT_MODE_DISABLED;
  } else {
    for (auto com : output_com_config_) {
      SBG_CONFIG(com.first, SBG_ECOM_OUTPUT_MODE_DISABLED);
    }
    sbg_sync_out_conf_[1].outputFunction = SBG_ECOM_SYNC_OUT_MODE_MAIN_LOOP;
  }

  sbgEComCmdSyncOutSetConf(&sbg_handle_, SBG_ECOM_SYNC_OUT_B,
                           &sbg_sync_out_conf_[1]);

  auto error = sbgEComCmdSettingsAction(&sbg_handle_, SBG_ECOM_SAVE_SETTINGS);
  response.success = (error == SBG_NO_ERROR);
  return true;
}

void SBGDriver::ReceiveEcomLog(SbgEComClass msg_class, SbgEComMsgId msg,
                               const SbgBinaryLogData *data) {
  auto it = ecom_callbacks_.find(msg);
  if (it != ecom_callbacks_.end()) {
    it->second(data);
  } else {
    ROS_WARN_THROTTLE(10, "Unhandled Ecom log %d", msg);
  }
}

void SBGDriver::RunOnce() {
  sbgEComHandle(&sbg_handle_);
}

void SBGDriver::EcomHandleImu(const SbgBinaryLogData * data) {
  ROS_INFO_THROTTLE(1, "IMU %d", data->imuData.timeStamp);
  sensor_msgs::Imu imu;
  pubs_[SBG_ECOM_LOG_IMU_DATA]->publish(imu);
}

void SBGDriver::EcomHandleUtc(const SbgBinaryLogData*) {
}

void SBGDriver::EcomHandleStatus(const SbgBinaryLogData* data) {
  ROS_INFO("Status %d", data->statusData.timeStamp);
}

void SBGDriver::EcomHandleEKFQuat(const SbgBinaryLogData*) {
}

void SBGDriver::EcomHandleMag(const SbgBinaryLogData*) {
}

void SBGDriver::EcomHandleEventA(const SbgBinaryLogData*) {
}

}  // namespace sbg
