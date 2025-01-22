#pragma once

#include <deque>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace common_data {
struct SocImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t time{0};  ///< In seconds.
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_ = Eigen::Vector3d::Zero();
  SocImuData() {}
  SocImuData(const uint64_t timestamp, const Eigen::Vector3d &angular_velocity,
             const Eigen::Vector3d &linear_acceleration)
            : time(timestamp), angular_velocity_(angular_velocity),
            linear_acceleration_(linear_acceleration){}
};
typedef std::deque<SocImuData, Eigen::aligned_allocator<SocImuData>> SocImuMeasurements;

} // namespace common_data