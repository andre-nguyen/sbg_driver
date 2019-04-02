//
#include <sbgEComLib.h>
#include "sbg_driver/sbg_driver.h"

#define SBG_HANDLE_ERROR_RET(...)  if(__VA_ARGS__ != SbgErrorCode::SBG_NO_ERROR) { return false; }

namespace sbg {

SBGDriver::SBGDriver(ros::NodeHandle &nh, ros::NodeHandle &pnh) noexcept :
  nh_(nh), pnh_(pnh)
{

}

SBGDriver::~SBGDriver() {
  sbgInterfaceSerialDestroy(&sbg_interface_);
}

bool SBGDriver::init() {
  SbgErrorCode errorCode;

  errorCode = sbgInterfaceSerialCreate(
      &sbg_interface_, "/dev/ttyUSB0", 921600);

  SBG_HANDLE_ERROR_RET(errorCode);

  errorCode = sbgEComInit(&sbg_handle_, &sbg_interface_);

  SBG_HANDLE_ERROR_RET(errorCode);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA,
                          SBG_ECOM_OUTPUT_MODE_MAIN_LOOP);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME,
                          SBG_ECOM_OUTPUT_MODE_DIV_200);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS,
                          SBG_ECOM_OUTPUT_MODE_DIV_200);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT,
                          SBG_ECOM_OUTPUT_MODE_MAIN_LOOP);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG,
                          SBG_ECOM_OUTPUT_MODE_DIV_4);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A,
                          SBG_ECOM_OUTPUT_MODE_EVENT_IN_A);

  sbgEComCmdOutputSetConf(&sbg_handle_, SBG_ECOM_OUTPUT_PORT_A,
                          SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B,
                          SBG_ECOM_OUTPUT_MODE_EVENT_IN_B);
}

}  // namespace sbg