#pragma once
#include <eigen3/Eigen/Dense>

class MagOutputData;
typedef std::shared_ptr<MagOutputData> MagOutputDataPtr;

class MagOutputData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MagOutputData(const uint64_t time, const float yaw):time_(time), yaw_(yaw) {
  };
  ~MagOutputData() {};

 public:
  uint64_t time_;
  float yaw_;
};
