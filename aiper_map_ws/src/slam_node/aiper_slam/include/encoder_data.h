#pragma once
#include "imu_data.h"
#include <eigen3/Eigen/Dense>
#include <memory>

class EncoderData;
typedef std::shared_ptr<EncoderData> EncoderDataPtr;

class EncoderData {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EncoderData(const uint64_t time_, const double v_l_, const double v_r_);
  ~EncoderData();
  bool interpolationAccGyroEuler(uint64_t ts, ImuMeasurements &imus);
  Eigen::Vector3d calDeltaEuler(EncoderDataPtr &last, EncoderDataPtr &curr);
  Eigen::Vector3d calAveEuler(EncoderDataPtr &last, EncoderDataPtr &curr);
  uint64_t getTime();
  double getVelocityLeft();
  double getVelocityRight();
  Eigen::Vector3d getAcc();
  Eigen::Vector3d getGyro();
  Eigen::Vector3d getEuler();


private:
  uint64_t time;
  double v_l;
  double v_r;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  Eigen::Vector3d euler;

};
