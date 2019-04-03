/**
 * @brief   Convert SBG Binary com logs to ROS messages
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    03/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#pragma once

#include <sbgECom.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sbg_msgs/ImuIntegral.h>
#include <sbg_msgs/Magnetometer.h>
#include <sbg_msgs/TriggerEvent.h>
#include <sbg_msgs/UtcTime.h>
#include <sensor_msgs/Imu.h>

namespace sbg {

inline void StampToRosHeader(const uint32 &timestamp, std_msgs::Header *header);

sensor_msgs::Imu ImuToRosImu(const SbgLogImuData &data);

geometry_msgs::PoseWithCovariance QuatToRosPose(const SbgLogEkfQuatData &data);

geometry_msgs::Quaternion QuatToRosQuat(const SbgLogEkfQuatData &data);

void QuatToRosQuatCov(const SbgLogEkfQuatData &data,
                      geometry_msgs::Quaternion *quat,
                      sensor_msgs::Imu::_orientation_covariance_type *cov);

sbg_msgs::TriggerEvent EventToRosEvent(const SbgLogEvent &data);

sbg_msgs::Magnetometer MagToRosMag(const SbgLogMag &data);

sbg_msgs::UtcTime UtcToRosUtc(const SbgLogUtcData &data);

}  // namespace sbg
