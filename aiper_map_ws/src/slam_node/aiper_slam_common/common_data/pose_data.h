#pragma once
#include <eigen3/Eigen/Dense>

namespace common_data {
class PoseData;
typedef std::shared_ptr<PoseData> PoseDataPtr;

class PoseData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum PoseType {
    TYPE_UNKNOWN  = 0,
    TYPE_STATIC = 1,
    TYPE_ROTATION = 2,
    TYPE_STRAIGHT = 3
  };
  PoseData(const uint64_t time, const Eigen::Quaterniond q, const Eigen::Vector3d p, const Eigen::Vector3d euler):
  time_(time), q_(q), p_(p), euler_(euler) {
    type_ = TYPE_UNKNOWN;
  };
  PoseData(){};
  ~PoseData() {};

 public:
  uint64_t time_;
  Eigen::Quaterniond q_;
  Eigen::Vector3d p_;
  Eigen::Vector3d euler_;
  PoseType type_;
};
} // namespace common_data