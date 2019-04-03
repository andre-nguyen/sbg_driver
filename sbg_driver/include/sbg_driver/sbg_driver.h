//
// Created by andre on 4/2/19.
//

#ifndef SBG_DRIVER_SBG_DRIVER_H_
#define SBG_DRIVER_SBG_DRIVER_H_

#include <sbgECom.h>
#include <sbgEComLib.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <functional>
#include <map>
#include <unordered_map>
#include <string>
#include <tuple>

namespace sbg {

class SBGDriver {
 public:
  SBGDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) noexcept;
  ~SBGDriver();
  bool Init();

  bool EnableStreamCallback(std_srvs::SetBoolRequest &request,
                            std_srvs::SetBoolResponse &response);

  void ReceiveEcomLog(SbgEComClass msg_class, SbgEComMsgId msg,
                      const SbgBinaryLogData *data);

 private:
  ros::NodeHandle nh_, pnh_;

  SbgInterface sbg_interface_;
  SbgEComHandle sbg_handle_;
  SbgEComSyncOutConf sbg_sync_out_conf_[2];

  std::unique_ptr<ros::Publisher> pubs_[SBG_ECOM_LOG_ECOM_NUM_MESSAGES];
  ros::ServiceServer enable_stream_service_;

  // ecom log -> output frequency
  std::map<SbgEComLog, SbgEComOutputMode> output_com_config_;

  // Ecom callbacks
  using EcomCallback = std::function<void(const SbgBinaryLogData*)>;
  std::unordered_map<SbgEComMsgId, EcomCallback> ecom_callbacks_;

  void EcomHandleImu(const SbgBinaryLogData*);
  void EcomHandleUtc(const SbgBinaryLogData*);
  void EcomHandleStatus(const SbgBinaryLogData*);
  void EcomHandleEKFQuat(const SbgBinaryLogData*);
  void EcomHandleMag(const SbgBinaryLogData*);
  void EcomHandleEventA(const SbgBinaryLogData*);
};

}  // namespace sbg

#endif  // SBG_DRIVER_SBG_DRIVER_H_
