/**
 * @brief
 * @author  Andre Phu-Van Nguyen <andre.phu-van.nguyen@ara-uas.com>
 * @date    03/04/19
 * @copyright Copyright (c) 2019 ARA Robotique. All rights reserved.
 */

#include "sbg_driver/sbg_ecom_ros_conversions.h"

#define SBG_ECOM_GET_CLOCK_STABLE(status) \
  ((status & SBG_ECOM_CLOCK_STABLE_INPUT) >> 0);
#define SBG_ECOM_GET_PPS_SYNC(status) \
  ((status & SBG_ECOM_CLOCK_UTC_SYNC) >> 0);

namespace sbg {

void StampToRosHeader(const uint32 &timestamp, std_msgs::Header *header) {
  const double kStampSeconds = static_cast<double>(timestamp) / 1e6;
  header->stamp = ros::Time(kStampSeconds);
  header->frame_id = "sbg_imu";
}

sensor_msgs::Imu ImuToRosImu(const SbgLogImuData &data) {
  sensor_msgs::Imu imu;
  StampToRosHeader(data.timeStamp, &imu.header);
  imu.linear_acceleration.x = data.accelerometers[0];
  imu.linear_acceleration.y = data.accelerometers[1];
  imu.linear_acceleration.z = data.accelerometers[2];
  imu.linear_acceleration_covariance[0] = -1;  // Unknown
  imu.angular_velocity.x = data.gyroscopes[0];
  imu.angular_velocity.y = data.gyroscopes[1];
  imu.angular_velocity.z = data.gyroscopes[2];
  imu.angular_velocity_covariance[0] = -1;  // Unknown

  imu.orientation_covariance[0] = -1;  // Unknown in this message

  return imu;
}

sbg_msgs::ImuIntegral ImuToImuInt(const SbgLogImuData &data) {
  sbg_msgs::ImuIntegral imu;
  imu.time_stamp = data.timeStamp;
  imu.imu.linear_acceleration.x = data.accelerometers[0];
  imu.imu.linear_acceleration.y = data.accelerometers[1];
  imu.imu.linear_acceleration.z = data.accelerometers[2];
  imu.imu.linear_acceleration_covariance[0] = -1;  // Unknown
  imu.imu.angular_velocity.x = data.gyroscopes[0];
  imu.imu.angular_velocity.y = data.gyroscopes[1];
  imu.imu.angular_velocity.z = data.gyroscopes[2];
  imu.imu.angular_velocity_covariance[0] = -1;  // Unknown
  imu.imu.orientation_covariance[0] = -1;  // Unknown in this message

  imu.temp = data.temperature;
  imu.delta_vel.x = data.deltaVelocity[0];
  imu.delta_vel.y = data.deltaVelocity[1];
  imu.delta_vel.z = data.deltaVelocity[2];
  imu.delta_angle.x = data.deltaAngle[0];
  imu.delta_angle.y = data.deltaAngle[1];
  imu.delta_angle.z = data.deltaAngle[2];

  return imu;
}

geometry_msgs::Quaternion QuatToRosQuat(const SbgLogEkfQuatData &data) {
  geometry_msgs::Quaternion quat;
  quat.w = data.quaternion[0];  // See Sbg Doc
  quat.x = data.quaternion[1];
  quat.y = data.quaternion[2];
  quat.z = data.quaternion[3];
  return quat;
}

geometry_msgs::PoseWithCovarianceStamped QuatToRosPose(const SbgLogEkfQuatData &data) {
  geometry_msgs::PoseWithCovarianceStamped p;
  p.pose.covariance[21] = data.eulerStdDev[0];
  p.pose.covariance[28] = data.eulerStdDev[0];
  p.pose.covariance[34] = data.eulerStdDev[0];
  p.pose.pose.orientation.w = data.quaternion[0];
  p.pose.pose.orientation.x = data.quaternion[1];
  p.pose.pose.orientation.y = data.quaternion[2];
  p.pose.pose.orientation.z = data.quaternion[3];
  return p;
}

void QuatToRosQuatCov(const SbgLogEkfQuatData &data,
                      geometry_msgs::Quaternion *quat,
                      sensor_msgs::Imu::_orientation_covariance_type *cov) {
  *quat = QuatToRosQuat(data);
  cov->elems[0] = data.eulerStdDev[0];
  cov->elems[3] = data.eulerStdDev[1];
  cov->elems[8] = data.eulerStdDev[2];
}

sbg_msgs::TriggerEvent EventToRosEvent(const SbgLogEvent &data) {
  sbg_msgs::TriggerEvent evt;

  StampToRosHeader(data.timeStamp, &evt.header);
  const double kStampSeconds = static_cast<double>(data.timeStamp) / 1e6;
  evt.stamp = ros::Time(kStampSeconds);

  return evt;
}

sbg_msgs::Magnetometer MagToRosMag(const SbgLogMag &data) {
  sbg_msgs::Magnetometer mag;
  StampToRosHeader(data.timeStamp, &mag.header);
  mag.mag.x = data.magnetometers[0];
  mag.mag.y = data.magnetometers[1];
  mag.mag.z = data.magnetometers[2];
  mag.accelerometer.x = data.accelerometers[0];
  mag.accelerometer.y = data.accelerometers[1];
  mag.accelerometer.z = data.accelerometers[2];

  return mag;
}

sbg_msgs::UtcTime UtcToRosUtc(const SbgLogUtcData &data) {
  sbg_msgs::UtcTime utc;
  StampToRosHeader(data.timeStamp, &utc.header);

  utc.clock_status.clock_stable = SBG_ECOM_GET_CLOCK_STABLE(data.status);
  utc.clock_status.clock_status = sbgEComLogUtcGetClockStatus(data.status);
  utc.clock_status.clock_pps_sync = SBG_ECOM_GET_PPS_SYNC(data.status);
  utc.clock_status.clock_utc_status =
      sbgEComLogUtcGetClockUtcStatus(data.status);

  return utc;
}

}  // namespace sbg
