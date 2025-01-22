#pragma once

#include <deque>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace common_data {
struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t time;  ///< In seconds.
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_acceleration_;
  Eigen::Vector3d euler_angle_;
  ImuData() {}
  ImuData(const uint64_t timestamp, const Eigen::Vector3d &angular_velocity,
          const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &euler_angle)
          : time(timestamp), angular_velocity_(angular_velocity),
            linear_acceleration_(linear_acceleration), euler_angle_(euler_angle){}
};
typedef std::deque<ImuData, Eigen::aligned_allocator<ImuData>> ImuMeasurements;
}  // namespace common_data