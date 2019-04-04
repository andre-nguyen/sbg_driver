// Copyright 2019 Andre

#include "sbg_driver/sequential_stamping.h"

namespace sbg {

SequentialTimestamping::SequentialTimestamping(const ros::NodeHandle &nh) :
  nh_(nh),
  last_imu_sbg_stamp_us_(0),
  is_started_(false),
  imu_fifo_(50), stamp_fifo_(50) {
}

}  // namespace sbg
