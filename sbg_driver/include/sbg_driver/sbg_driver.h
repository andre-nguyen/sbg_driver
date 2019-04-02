//
// Created by andre on 4/2/19.
//

#ifndef SBG_DRIVER_SBG_DRIVER_H
#define SBG_DRIVER_SBG_DRIVER_H

#include <sbgECom.h>
#include <ros/ros.h>

namespace sbg {

class SBGDriver {
 public:
  SBGDriver(ros::NodeHandle &nh, ros::NodeHandle &pnh) noexcept;
  ~SBGDriver();
  bool init();


 private:
  ros::NodeHandle nh_, pnh_;

  SbgInterface sbg_interface_;
  SbgEComHandle sbg_handle_;

  enum OutputLogs {
    OLOG_IMU,
    OLOG_UTC,
    OLOG_STATUS,
    OLOG_QUAT,
    OLOG_MAG,
    OLOG_EVENTA,
    OLOG_EVENTB,
    OLOG_COUNT
  };

  std::unique_ptr<ros::Publisher> pubs_[OLOG_COUNT];
};

}  // namespace sbg

#endif //SBG_DRIVER_SBG_DRIVER_H
