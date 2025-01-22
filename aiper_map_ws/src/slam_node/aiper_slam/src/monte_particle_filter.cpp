//
// Created by jeffrey on 2023/10/10.
//
#include "monte_particle_filter.h"
#include "common_tool.h"
#include "tic_toc.h"
#include "optimized_para.h"
#include "map_modules/buildMapWithTwoSonar.h"
#include <cmath>
//#include "map_modules/icp_match.h"


static std::mutex icp_carrier_lock_;
std::deque<uint64_t> circle_timestamp_buffer_;
//class IcpMatcher;
//std::shared_ptr<IcpMatcher> IcpMatcher_ptr_;
//class HJ_slam::HJ_mapping::BuildMapWithTwoSonar;
//typedef std::shared_ptr<HJ_slam::HJ_mapping::BuildMapWithTwoSonar> BuildMapWithTwoSonarPtr_;
//static HJ_slam::HJ_mapping::BuildMapWithTwoSonarPtr build_map_ptr_ = nullptr;

namespace HJ_slam {
namespace HJ_tracking {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const TrackingConfigParameters &config_parameters)
        : tracking_config_parameters_(config_parameters) {
  yaw_kf_ = std::make_shared<Yaw_Kalman_Filter>();
#ifdef Save_File
  ofs_pose.open("./hj_slam_pose.txt", std::fstream::out);
#endif
  Twb_mcu_.block<1, 3>(0, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_imu_[0], tracking_config_parameters_.Twb_mcu_imu_[1], tracking_config_parameters_.Twb_mcu_imu_[2]);
  Twb_mcu_.block<1, 3>(1, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_imu_[3], tracking_config_parameters_.Twb_mcu_imu_[4], tracking_config_parameters_.Twb_mcu_imu_[5]);
  Twb_mcu_.block<1, 3>(2, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_imu_[6], tracking_config_parameters_.Twb_mcu_imu_[7], tracking_config_parameters_.Twb_mcu_imu_[8]);
  Twb_euler_.block<1, 3>(0, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_euler_[0], tracking_config_parameters_.Twb_mcu_euler_[1], tracking_config_parameters_.Twb_mcu_euler_[2]);
  Twb_euler_.block<1, 3>(1, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_euler_[3], tracking_config_parameters_.Twb_mcu_euler_[4], tracking_config_parameters_.Twb_mcu_euler_[5]);
  Twb_euler_.block<1, 3>(2, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_mcu_euler_[6], tracking_config_parameters_.Twb_mcu_euler_[7], tracking_config_parameters_.Twb_mcu_euler_[8]);
  Twb_soc_.block<1, 3>(0, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_soc_imu_[0], tracking_config_parameters_.Twb_soc_imu_[1], tracking_config_parameters_.Twb_soc_imu_[2]);
  Twb_soc_.block<1, 3>(1, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_soc_imu_[3], tracking_config_parameters_.Twb_soc_imu_[4], tracking_config_parameters_.Twb_soc_imu_[5]);
  Twb_soc_.block<1, 3>(2, 0) = Eigen::Vector3d(tracking_config_parameters_.Twb_soc_imu_[6], tracking_config_parameters_.Twb_soc_imu_[7], tracking_config_parameters_.Twb_soc_imu_[8]);
//  earth_rotation_speed_ = tracking_config_parameters_.earth_rotation_speed_;
  g_ = Eigen::Vector3d(0.0, 0.0, config_parameters.earth_gravity_);
  accel_bias_ = Eigen::Vector3d(config_parameters.accelerometer_bias_[0], config_parameters.accelerometer_bias_[1], config_parameters.accelerometer_bias_[2]);
  gyro_bias_ = Eigen::Vector3d(config_parameters.gyro_bias_[0], config_parameters.gyro_bias_[1], config_parameters.gyro_bias_[2]);
  setCovarianceP(tracking_config_parameters_.position_error_prior_std_,
                 tracking_config_parameters_.velocity_error_prior_std_,
                 tracking_config_parameters_.rotation_error_prior_std_,
                 tracking_config_parameters_.gyro_bias_error_prior_std_,
                 tracking_config_parameters_.accelerometer_bias_error_prior_std_);

  setCovarianceR(tracking_config_parameters_.encoder_position_x_std_,
                 tracking_config_parameters_.encoder_position_y_std_,
                 tracking_config_parameters_.encoder_position_z_std_,
                 tracking_config_parameters_.encoder_velocity_x_std_,
                 tracking_config_parameters_.encoder_velocity_y_std_,
                 tracking_config_parameters_.encoder_velocity_z_std_);

  setCovarianceQ(tracking_config_parameters_.gyro_noise_std_, tracking_config_parameters_.accelerometer_noise_std_);

  X_.setZero();
  F_.setZero();
  C_.setIdentity();
  G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
  G_.block<3, 3>(INDEX_MEASUREMENT_VEL, INDEX_MEASUREMENT_VEL) = Eigen::Matrix3d::Identity();
}

void ErrorStateKalmanFilter::setCovarianceQ(double gyro_noise, double accel_noise) {
  Q_.setZero();
  Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
  Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
}

void ErrorStateKalmanFilter::setCovarianceR(double position_x_std, double position_y_std, double position_z_std,
                                            double velocity_x_std, double velocity_y_std, double velocity_z_std) {
  R_.setZero();
  R_(0, 0) = position_x_std * position_x_std;
  R_(1, 1) = position_y_std * position_y_std;
  R_(2, 2) = position_z_std * position_z_std;
  R_(3, 3) = velocity_x_std * velocity_x_std;
  R_(4, 4) = velocity_y_std * velocity_y_std;
  R_(5, 5) = velocity_z_std * velocity_z_std;
}

void ErrorStateKalmanFilter::setCovarianceP(double posi_noise, double velocity_noise, double ori_noise,
                                            double gyro_noise, double accel_noise) {
  P_.setZero();
  P_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
  P_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velocity_noise * velocity_noise;
  P_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise * ori_noise;
  P_.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) =
          Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
  P_.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) =
          Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
}

SocImuData ErrorStateKalmanFilter::interpolationAcc(uint64_t ts, SocImuMeasurements &imus) {
  SocImuData target_encoder_;
  target_encoder_.time = ts;
  target_encoder_.angular_velocity_ = Eigen::Vector3d(0, 0, 0);
  target_encoder_.linear_acceleration_ = Eigen::Vector3d(0, 0, 0);

  SocImuData m_low;
  SocImuData m_high;
  bool find_imu_older = false;
  bool find_imu_newer = false;

  for (std::deque<SocImuData, Eigen::aligned_allocator<SocImuData>>::reverse_iterator rit = imus.rbegin(); rit!=imus.crend(); ++rit) {
    if ((*rit).time > ts) {
      m_high = (*rit);
      find_imu_newer = true;
      break;
    }
  }
  for (std::deque<SocImuData, Eigen::aligned_allocator<SocImuData>>::iterator rit = imus.begin(); rit != imus.end(); ++rit) {
    if ((*rit).time < ts) {
      m_low = (*rit);
      find_imu_older = true;
      break;
    }
  }
  if (find_imu_newer == true && find_imu_older == true) {
    double k1 = (static_cast<double>(ts - m_low.time)) / (static_cast<double>(m_high.time - m_low.time));
    double k2 = (static_cast<double>(m_high.time - ts)) / (static_cast<double>(m_high.time - m_low.time));
    target_encoder_.linear_acceleration_ = k1 * m_high.linear_acceleration_ + k2 * m_low.linear_acceleration_;
    target_encoder_.angular_velocity_ = k1 * m_high.angular_velocity_ + k2 * m_low.angular_velocity_;

  }
  else if (find_imu_newer == false && find_imu_older == true){
    target_encoder_.linear_acceleration_ = m_low.linear_acceleration_;
    target_encoder_.angular_velocity_ =  m_low.angular_velocity_;


  }
  else if (find_imu_newer == true && find_imu_older == false){
    target_encoder_.linear_acceleration_ = m_high.linear_acceleration_;
    target_encoder_.angular_velocity_ =  m_high.angular_velocity_;

  }
  else {
    HJ_WARN("interpolationAcc is error!");
  }
  return target_encoder_;
}

bool ErrorStateKalmanFilter::doMagInitial(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, MagData &mag, bool &is_mag) {
  if (is_mag && !imus.empty()) {
    Eigen::Matrix<double, 7, 1> temp;
    temp << curr_encoder->getTime(), imus[0].euler_angle_[0], imus[0].euler_angle_[1], imus[0].euler_angle_[2],
            static_cast<double>(mag.mag_x), static_cast<double>(mag.mag_y), static_cast<double>(mag.mag_z);
    time_euler_mag.push_back(temp);
    curr_imu_data_ = imus[0];
    if (is_first_mag) {
      last_imu_data_ = imus[0];
      is_first_mag = false;
    }
    else {
      Eigen::Vector3d delta_angle = computeDeltaRotation(last_imu_data_, curr_imu_data_);
      if (delta_angle[2] < 0.0) {
        HJ_ERROR("last error = %f, %f, %f", last_imu_data_.euler_angle_[0], last_imu_data_.euler_angle_[1], last_imu_data_.euler_angle_[2]);
        HJ_ERROR("curr error= %f, %f, %f", curr_imu_data_.euler_angle_[0], curr_imu_data_.euler_angle_[1], curr_imu_data_.euler_angle_[2]);
        HJ_ERROR("delta error= %f, %f, %f", delta_angle[0], delta_angle[1], delta_angle[2]);
        HJ_ERROR("calib mag data needs turn left but it turns right");
      }
      init_yaw_sum += fabs(delta_angle[2]);
      last_imu_data_ = curr_imu_data_;
    }
  }
#ifdef With_mag
  const double yaw_max_value = 700.0;
#else
  const double yaw_max_value = 0.0;
#endif
  if (init_yaw_sum < yaw_max_value) {
    return false;
  }
  return true;
}

void ErrorStateKalmanFilter::filterImuData(ImuMeasurements &imus) {
  for (auto it = imus.begin(); it != imus.end();) {
    bool if_condition = it->euler_angle_[0] > 180.0 || it->euler_angle_[0] < -180.0 ||
                        it->euler_angle_[1] > 180.0 || it->euler_angle_[1] < -180.0 ||
                        it->euler_angle_[2] > 180.0 || it->euler_angle_[2] < -180.0;
    if (if_condition) {
      HJ_ERROR("delete imu beacuse condition 1");
      HJ_ERROR("imu: = %lf, %lf, %lf, %ld", it->euler_angle_[0], it->euler_angle_[1], it->euler_angle_[2], it->time);
      it = imus.erase(it);
    } else {
      ++it;
    }
  }

  bool need_deleted = false;
  std::vector<int> deleted_candidate_id;
  for (int j = 0; j < imus.size() - 1; ++j) {
    Eigen::Vector3d delta_rotation = computeDeltaRotation(imus[j], imus[j + 1]);
    if (fabs(delta_rotation[0]) > 1.0 || fabs(delta_rotation[1]) > 1.0 || fabs(delta_rotation[2]) > 1.0) {
      need_deleted = true;
      deleted_candidate_id.push_back(j);
    }
  }
  std::vector<int> to_deleted_id;
  if (need_deleted) {
    if (deleted_candidate_id.size() == 1) {
      if (deleted_candidate_id[0] == 0) {
        to_deleted_id.push_back(0);
      }
      if (deleted_candidate_id[0] == imus.size() - 2) {
        to_deleted_id.push_back(imus.size() - 1);
      }
    }
    else if (deleted_candidate_id.size() == 2) {
      to_deleted_id.push_back(deleted_candidate_id[1]);
    }
    else {
      HJ_ERROR("to deleted imu is too much");
    }
  }

  if (to_deleted_id.size() == 1) {
    int deleted_id = to_deleted_id[0];
    int index = 0;
    for (auto it = imus.begin(); it != imus.end(); ++index) {

      if (deleted_id == index) {
        HJ_ERROR("delete imu beacuse condition 2");
        HJ_ERROR("imu: = %lf, %lf, %lf, %ld", it->euler_angle_[0], it->euler_angle_[1], it->euler_angle_[2], it->time);
        it = imus.erase(it);
      } else {
        ++it;
      }
    }
  }

}


bool ErrorStateKalmanFilter::initialize(EncoderDataPtr &curr_encoder, ImuMeasurements &imus,
                                        MagData &mag, bool &is_mag, bool &is_mag_success_init) {
  TicToc trackingInitialTime;
  X_ = TypeVectorX::Zero();
  checkImuAxis(imus);
  bool is_initialized = setInitialPose(curr_encoder, imus, mag, is_mag, is_mag_success_init);
  if (!is_initialized) {
    return false;
  }
  /*
  if (is_mag){
    MagData curr_encoder_mag = mag;
    curr_encoder_mag.calculateMagDegree(mag.mag_x, mag.mag_y,
                                        tracking_config_parameters_.magnetic_offset_x_, tracking_config_parameters_.magnetic_offset_y_);
    init_mag_theta_ = curr_encoder_mag.yaw;
  }
  else{
    PoseData pose(curr_encoder_data.time, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return pose;
  }
    */
  /*
  pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) + t_bm_x9;
  Eigen::Quaterniond curr_q = Eigen::Quaterniond(getPose().topLeftCorner<3, 3>());
  curr_q.normalized();
  Eigen::Vector3d curr_position = getPose().topRightCorner<3, 1>();
  Eigen::Vector3d curr_angle = Quaternion2EulerAngles(curr_q);
  Eigen::Vector3d curr_angle_temp = Eigen::Vector3d(curr_angle[2], curr_angle[1], curr_angle[0]);
   */
//#ifdef X9
  Eigen::Vector3d t_external = Eigen::Vector3d::Zero();
#ifdef X9
  t_external = t_bm_x9;
#endif
#ifdef T1_pro
  t_external = t_bm_t1_pro;
#endif
  pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) + t_external;
  lastEncoder = curr_encoder;
  need_reset = false;
  double time_initial = trackingInitialTime.toc();
  HJ_INFO("slam initial is successfully cost time %lf", time_initial);
  return true;

}

void ErrorStateKalmanFilter::softReset() {
  resetAllState();

}

void ErrorStateKalmanFilter::resetAllState() {
  X_.setZero();
  F_.setZero();
  C_.setIdentity();
  G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
  G_.block<3, 3>(INDEX_MEASUREMENT_VEL, INDEX_MEASUREMENT_VEL) = Eigen::Matrix3d::Identity();

  while (!time_angle_pose_front_ultra.empty()) {
    time_angle_pose_front_ultra.pop_front();
  }
  while (!angle_pose.empty()) {
    angle_pose.pop_front();
  }
  optimization_count = 0;
  velocity_ = Eigen::Vector3d::Zero();
  pose_ = Eigen::Matrix4d::Identity();
}

void ErrorStateKalmanFilter::checkImuAxis(ImuMeasurements &imus) {
  for (int i = imus.size() - 1; i >= 0; --i) {
    Eigen::Vector3d acc = imus[i].linear_acceleration_;
    Eigen::Vector3d gyro = imus[i].angular_velocity_;
    Eigen::Vector3d euler = imus[i].euler_angle_;
    Eigen::Matrix<double, 3, 1> acc_mat = acc;
    Eigen::Matrix<double, 3, 1> gyro_mat = gyro;
    Eigen::Matrix<double, 3, 1> euler_mat = euler;
    Eigen::Matrix<double, 3, 1> acc_world = Twb_mcu_ * acc_mat;
    Eigen::Matrix<double, 3, 1> gyro_world = Twb_mcu_ * gyro_mat;
    Eigen::Matrix<double, 3, 1> euler_world = Twb_euler_ * euler_mat;
    imus[i].linear_acceleration_ = Eigen::Vector3d(acc_world(0, 0), acc_world(1, 0), acc_world(2, 0));
    imus[i].angular_velocity_ = Eigen::Vector3d(gyro_world(0, 0), gyro_world(1, 0), gyro_world(2, 0));
    imus[i].euler_angle_ = Eigen::Vector3d(euler_world(0, 0), euler_world(1, 0), euler_world(2, 0));
  }
}

void ErrorStateKalmanFilter::checkSocImuAxis(SocImuMeasurements &soc_imus) {
  for (int i = soc_imus.size() - 1; i >= 0; --i) {
    Eigen::Vector3d acc = soc_imus[i].linear_acceleration_;
    Eigen::Vector3d gyro = soc_imus[i].angular_velocity_;
    Eigen::Matrix<double, 3, 1> acc_mat = acc;
    Eigen::Matrix<double, 3, 1> gyro_mat = gyro;
    Eigen::Matrix<double, 3, 1> acc_world = Twb_soc_ * acc_mat;
    Eigen::Matrix<double, 3, 1> gyro_world = Twb_soc_ * gyro_mat;
    soc_imus[i].linear_acceleration_ = Eigen::Vector3d(acc_world(0, 0), acc_world(1, 0), acc_world(2, 0));
    soc_imus[i].angular_velocity_ = Eigen::Vector3d(gyro_world(0, 0), gyro_world(1, 0), gyro_world(2, 0));
  }
}

PoseDataPtr ErrorStateKalmanFilter::process_three_dof(EncoderDataPtr &curr_encoder, ImuMeasurements &imus) {
  checkImuAxis(imus);
//  currEncoder = curr_encoder;
  curr_encoder->interpolationAccGyroEuler(curr_encoder->getTime(), imus);
  Eigen::Quaterniond temp_q  = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d temp_pos  = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d temp_angle = Eigen::Vector3d(curr_encoder->getEuler()[0] * kDegree2Radian, curr_encoder->getEuler()[1] * kDegree2Radian, curr_encoder->getEuler()[2] * kDegree2Radian);
  auto pose = std::make_shared<PoseData>(curr_encoder->getTime(), temp_q, temp_pos, temp_angle);

  return pose;

}

void ErrorStateKalmanFilter::collectMagData(EncoderDataPtr &curr_encoder, Eigen::Vector3d &mag, bool &is_mag, bool &collect_ending) {
  if (!collect_ending && is_mag) {
    Eigen::Matrix<double, 7, 1> temp;
    temp << static_cast<double>(curr_encoder->getTime()) * 1e-6, curr_encoder->getEuler()[0] * kDegree2Radian, curr_encoder->getEuler()[1] * kDegree2Radian,
            curr_encoder->getEuler()[2] * kDegree2Radian, mag[0], mag[1], mag[2];
    time_euler_mag.push_back(temp);
  }

}

MagOutputDataPtr ErrorStateKalmanFilter::estimateMagYaw(EncoderDataPtr &curr_encoder, MagData &mag, bool &is_mag) {
  double diff_time = fabs(static_cast<double>(curr_encoder->getTime() - mag.time) * 1e-6);
  Eigen::Matrix3d Tw_mag;
  Tw_mag << 0, -1, 0, -1, 0, 0, 0, 0, -1;
  Eigen::Matrix<double, 3, 1> mag_mat;
  mag_mat << mag.mag_x, mag.mag_y, mag.mag_z;

  Eigen::Quaterniond quaternion = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(curr_encoder->getEuler()[1] * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(curr_encoder->getEuler()[0] * kDegree2Radian, Eigen::Vector3d::UnitX());
  quaternion.normalize();
  Eigen::Vector3d mag_check = quaternion.toRotationMatrix() * Tw_mag * mag_mat;

  float result_yaw = -1 * mag.calculateMagDegree(mag_check[0], mag_check[1], mag_offset_[0], mag_offset_[1]);

  auto mag_output = std::make_shared<MagOutputData>(curr_encoder->getTime(), result_yaw);
  return mag_output;
}

PoseDataPtr ErrorStateKalmanFilter::process(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus,
                                         MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra) {
  TicToc trackingAllTime;
  TicToc trackingInitialTime;

  currEncoder = curr_encoder;

  TicToc trackingPredictTime;
  checkImuAxis(imus);
  doPredicting(imus);

#ifdef Aiper_surface_vessel
  checkSocImuAxis(soc_imus);
  angle_soc_imu += doPredicting(soc_imus);
#endif

  double time_part2 = trackingPredictTime.toc();

  TicToc trackingCorrectTime;
  correct();
  double time_part3 = trackingCorrectTime.toc();

  TicToc trackingOtherTime;
  lastEncoder = currEncoder;

  auto curr_rotation = getPose().topLeftCorner<3, 3>();
  auto curr_q = Eigen::Quaterniond(curr_rotation);
  uint64_t time = curr_encoder->getTime();
  Eigen::Vector3d angle = Quaternion2EulerAngles(curr_q);//yaw, pitch, roll
  Eigen::Vector3d position = getPose().topRightCorner<3, 1>();
  double time_part4 = trackingOtherTime.toc();

  //check and use magnetic to optimize
  TicToc trackingUseMagTime;
  Eigen::Vector3d angle_fusion = Eigen::Vector3d(angle[0], angle[1], angle[2]);
  if (tracking_config_parameters_.use_magnetic_model_ && is_mag) {
    float yaw_fusion = doMagFusion(mag, angle); //degree
    angle_fusion[0] = yaw_fusion * kDegree2Radian;
  }
  else {
    angle_fusion[0] = angle[0];
  }
  double time_part5 = trackingUseMagTime.toc();

  TicToc trackingCorrectByUltraTime;
  bool is_corrected_by_ultra = correctByUltra(ultra_measurement, get_ultra, angle_fusion, position);
  if (is_corrected_by_ultra) {
    position[0] = pose_(0, 3);
    position[1] = pose_(1, 3);
  }

//  std::vector<Eigen::Matrix4d> icp_trans_list;
//  bool get_icp_trans = IcpMatcher_ptr_->getIcpTransform(icp_trans_list);
//
//  if (get_icp_trans) {
//    Eigen::Matrix4d icp_transform = icp_trans_list[0];
//
//    auto icp_q = Eigen::Quaterniond(icp_transform.topLeftCorner<3, 3>());
//    auto icp_angle = Quaternion2EulerAngles(icp_q);
//    double d_ = sqrt(icp_transform(0, 3) * icp_transform(0, 3) + icp_transform(1, 3) * icp_transform(1, 3) +
//            icp_transform(2, 3) * icp_transform(2, 3));
//    if (fabs(icp_angle[0] * kRadian2Degree) < 10.0 && d_ < 0.25) {
//      HJ_CHECK_EQ2(-1, 2);
//      pose_ = icp_transform * pose_;
//      time_angle_pose_front_ultra.clear();
//
//    }
//  }

  // correted by relocalization
  if (is_relocalization_success_ == true) {
    // Eigen::Vector3d curr_euler_angle_ = curr_imu_data_.euler_angle_ * kDegree2Radian;
    // double curr_theta = calDeltaThetaBetweenInitAndCurrent(init_imu_theta_[2], curr_imu_data_.euler_angle_[2]);
    // Eigen::Vector3d tmp_curr_theta = Eigen::Vector3d(curr_euler_angle_[0], curr_euler_angle_[1], curr_theta * kDegree2Radian);
    // Eigen::Vector3d curr_theta_corrected = tmp_curr_theta + relocalization_diff_angle_;
    // HJ_INFO("curr_theta_corrected: %lf, %lf, %lf", curr_theta_corrected[0] * kRadian2Degree, curr_theta_corrected[1] * kRadian2Degree, curr_theta_corrected[2] * kRadian2Degree);
    // if (curr_theta_corrected[2] > M_PI) {
    //   curr_theta_corrected[2] -= 2 * M_PI;
    // }
    // if (curr_theta_corrected[2] < -M_PI) {
    //   curr_theta_corrected[2] += 2 * M_PI;
    // }
    // HJ_INFO("current theta corrected: %lf, %lf, %lf", curr_theta_corrected[0] * kRadian2Degree, curr_theta_corrected[1] * kRadian2Degree, curr_theta_corrected[2] * kRadian2Degree);
    // auto curr_quaternion = Eigen::AngleAxisd(curr_theta_corrected[2], Eigen::Vector3d::UnitZ()) *
    //                        Eigen::AngleAxisd(curr_theta_corrected[1], Eigen::Vector3d::UnitY()) *
    //                        Eigen::AngleAxisd(curr_theta_corrected[0], Eigen::Vector3d::UnitX());
    // curr_quaternion.normalize();
    // pose_.block<3, 3>(0, 0) = curr_quaternion.toRotationMatrix();
    pose_.block<3, 1>(0, 3) = relocalization_position_;

    relocalization_position_ = Eigen::Vector3d::Zero();
    is_relocalization_success_ = false;
    time_angle_pose_front_ultra.clear();
  }

  Eigen::Vector3d angle_fusion_temp = Eigen::Vector3d(angle_fusion[2], angle_fusion[1], angle_fusion[0]);
#ifdef Aiper_surface_vessel
  Eigen::Quaterniond quaternion_soc = Eigen::AngleAxisd(angle_soc_imu[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(angle_soc_imu[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(angle_soc_imu[0], Eigen::Vector3d::UnitX());
  quaternion_soc.normalize();
  auto angle_soc = Quaternion2EulerAngles(quaternion_soc);
  auto pose = std::make_shared<PoseData>(time, curr_q, position, angle_fusion_temp);
#else
  auto pose = std::make_shared<PoseData>(time, curr_q, position, angle_fusion_temp);
#endif
  double time_part6 = trackingCorrectByUltraTime.toc();

#ifdef Save_File
  ofs_pose << std::setprecision(18) << curr_encoder_data.time * 1e-6 << " " << position[0] << " " << position[1] << " " << position[2] << " "
  << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << std::endl;
#endif
  double time_all = trackingAllTime.toc();
  HJ_INFO("#updateState: %lf,%lf,%lf,%lf,%lf,%lf,%ld", time_all, time_part2, time_part3, time_part4, time_part5, time_part6, curr_encoder->getTime());
  return pose;
}

void ErrorStateKalmanFilter::getFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
  F = Ft_;
  G = G_;
  Y = Y_;
}

bool ErrorStateKalmanFilter::correct() {

  double encoder_scale_v = tracking_config_parameters_.encoder_scale_;
//  double encoder_length = tracking_config_parameters_.encoder_rc_;

  double v_l = static_cast<double>(currEncoder->getVelocityLeft()) * encoder_scale_v;
  double v_r = static_cast<double>(currEncoder->getVelocityRight()) * encoder_scale_v;

  double delta_encoder_t = static_cast<double>(currEncoder->getTime() - lastEncoder->getTime()) * 1e-6;

  auto last_q = Eigen::Quaterniond(last_pose_.topLeftCorner<3, 3>());
  auto last_angle = Quaternion2EulerAngles(last_q);

  Eigen::Vector3d last_angle_s = Eigen::Vector3d(last_angle[2], last_angle[1], last_angle[0]);
  last_angle_s[2] = last_angle_s[2] < 0 ? last_angle_s[2] + 2 * 3.1416 : last_angle_s[2];
  Eigen::Vector3d delta_theta = currEncoder->calDeltaEuler(lastEncoder, currEncoder);

  Eigen::Vector3d delta_theta_radian = delta_theta * kDegree2Radian;

  auto curr_ave_theta = last_angle_s + delta_theta_radian / 2;

  double px_w = last_pose_(0,3) + 0.5 * (v_l + v_r) * cos(curr_ave_theta[2]) * delta_encoder_t * cos(-curr_ave_theta[1]);
  double py_w = last_pose_(1,3) + 0.5 * (v_l + v_r) * sin(curr_ave_theta[2]) * delta_encoder_t;
  double pz_w = last_pose_(2,3) + 0.5 * (v_l + v_r) * -sin(0.0) * delta_encoder_t;
  double vx_w = 0.5 * (v_l + v_r) * cos(curr_ave_theta[2]) * cos(-curr_ave_theta[1]);
  double vy_w = 0.5 * (v_l + v_r) * sin(curr_ave_theta[2]);
  double vz_w = 0.5 * (v_l + v_r) * -sin(0.0);

  Eigen::VectorXd m = Eigen::VectorXd::Zero(6);
  m << px_w, py_w, pz_w, vx_w, vy_w, vz_w;
  Eigen::VectorXd m_predict = Eigen::VectorXd::Zero(6);
  m_predict << pose_(0,3), pose_(1,3), pose_(2,3),
               velocity_(0), velocity_(1), velocity_(2);


  Y_ = m - m_predict;
  K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse();

  P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;

  K_.block<1, 6>(6, 0) = Eigen::MatrixXd::Zero(1, 6);
  K_.block<1, 6>(7, 0) = Eigen::MatrixXd::Zero(1, 6);
  K_.block<1, 6>(8, 0) = Eigen::MatrixXd::Zero(1, 6);

  X_ = X_ + K_ * (Y_ - G_ * X_);

  eliminateError();
//  printf("#accel_bias_result: %f, %f, %f, %ld\n", accel_bias_[0], accel_bias_[1], accel_bias_[2], curr_encoder_data_.time);
//  printf("#gyro_bias_result: %f, %f, %f, %ld\n", gyro_bias_[0], gyro_bias_[1], gyro_bias_[2], curr_encoder_data_.time);
  resetState();

  return true;
}

void ErrorStateKalmanFilter::predict(const ImuData &curr_imu_data) {
  curr_imu_data_ = curr_imu_data;
  Eigen::Vector3d w_in = Eigen::Vector3d::Zero();

  if (tracking_config_parameters_.use_earth_model_) {
//      w_in = ComputeNavigationFrameAngularVelocity(); // 时刻 m-1 -> m 地球转动引起的导航系转动角速度
  }
  Eigen::Vector3d delta_rotation(0.0f, 0.0f, 0.0f);
  updateOdomEstimation(w_in, delta_rotation);//求出世界系下的P Q V
  
  double delta_t = static_cast<double>(curr_imu_data_.time - last_imu_data_.time) * 1e-6;
  Eigen::Vector3d curr_accel = pose_.block<3, 3>(0, 0) * curr_imu_data.linear_acceleration_;

  updateErrorState(delta_t, curr_accel, w_in);

  last_imu_data_ = curr_imu_data_;
}

#ifdef Aiper_surface_vessel
Eigen::Vector3d ErrorStateKalmanFilter::predict(const SocImuData &curr_imu_data) {
  curr_soc_imu_data_ = curr_imu_data;
  Eigen::Vector3d w_in = Eigen::Vector3d::Zero();

  Eigen::Vector3d delta_rotation(0.0f, 0.0f, 0.0f);
  updateSocRotation(w_in, delta_rotation);//求出世界系下的P Q V
  last_soc_imu_data_ = curr_soc_imu_data_;
  return delta_rotation;
}
#endif

void ErrorStateKalmanFilter::updateErrorState(double t, const Eigen::Vector3d &accel, const Eigen::Vector3d &w_in_n) {
  Eigen::Matrix3d F_23 = BuildSkewSymmetricMatrix(accel);
  Eigen::Matrix3d F_33 = -BuildSkewSymmetricMatrix(w_in_n);

  F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
  F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = F_33;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3, 3>(0, 0);
  F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -pose_.block<3, 3>(0, 0);
  B_.block<3, 3>(INDEX_STATE_VEL, 3) = pose_.block<3, 3>(0, 0);
  B_.block<3, 3>(INDEX_STATE_ORI, 0) = -pose_.block<3, 3>(0, 0);

  TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
  TypeMatrixB Bk = B_ * t;

  // 用于可观测性分析
  Ft_ = F_ * t;

  X_ = Fk * X_;
  P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();
}

void ErrorStateKalmanFilter::updateOdomEstimation(const Eigen::Vector3d &w_in, Eigen::Vector3d &delta_rotation) {
//  const double last_imu_time_ = static_cast<double>(last_imu_data_.time) * 1e-6;
//  const double curr_imu_time_ = static_cast<double>(curr_imu_data_.time) * 1e-6;
//  const double delta_t = curr_imu_time_ - last_imu_time_;

  delta_rotation = computeDeltaRotation(last_imu_data_, curr_imu_data_);//degree
  if (fabs(delta_rotation[0]) > 1.0 || fabs(delta_rotation[1]) > 1.0 || fabs(delta_rotation[2]) > 1.0) {
    HJ_ERROR("delta rotation is too large, last timestamp is %ld, curr timestamp is %ld",
             last_imu_data_.time, curr_imu_data_.time);
    HJ_ERROR("delta rotation is too large %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
             last_imu_data_.euler_angle_[0], last_imu_data_.euler_angle_[1], last_imu_data_.euler_angle_[2],
             curr_imu_data_.euler_angle_[0], curr_imu_data_.euler_angle_[1], curr_imu_data_.euler_angle_[2],
             delta_rotation[0], delta_rotation[1], delta_rotation[2]);
  }

//  const Eigen::Vector3d phi_in = w_in * delta_t;
//  const Eigen::AngleAxisd angle_axisd(phi_in.norm(), phi_in.normalized());
//  const Eigen::Matrix3d R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose();

  Eigen::Matrix3d curr_R; // R_n_m m时刻的旋转
  Eigen::Matrix3d last_R; // C_n_m_1 m-1时刻的旋转
  //ComputeOrientation(delta_rotation, R_nm_nm_1, curr_R, last_R);//求Rwb
  computeOrientationEuler(curr_R, last_R, delta_rotation);//求Rwb

  Eigen::Vector3d curr_vel; // 当前时刻导航系下的速度
  Eigen::Vector3d last_vel; // 上一时刻导航系下的速度
  computeVelocity(last_R, curr_R, last_imu_data_, curr_imu_data_, last_vel, curr_vel);

  computePosition(last_R, curr_R, last_vel, curr_vel, last_imu_data_, curr_imu_data_);
}

#ifdef Aiper_surface_vessel
void ErrorStateKalmanFilter::updateSocRotation(const Eigen::Vector3d &w_in, Eigen::Vector3d &delta_rotation) {
  const double last_imu_time_ = static_cast<double>(last_soc_imu_data_.time) * 1e-6;
  const double curr_imu_time_ = static_cast<double>(curr_soc_imu_data_.time) * 1e-6;

  const double delta_t = curr_imu_time_ - last_imu_time_;
  if (delta_t > 10.0)
    return;
  delta_rotation = computeSocDeltaRotation(last_soc_imu_data_, curr_soc_imu_data_);

}
#endif


Eigen::Vector3d ErrorStateKalmanFilter::computeDeltaRotation(const ImuData &imu_data_0, const ImuData &imu_data_1) {
  Eigen::Vector3d last_euler = imu_data_0.euler_angle_;
  Eigen::Vector3d curr_euler = imu_data_1.euler_angle_;
  Eigen::Vector3d delta_euler = Eigen::Vector3d::Zero();
  delta_euler = curr_euler - last_euler;//0.0, 0.01,0.03
  bool flag = last_euler[2] * curr_euler[2] >= 0.0 ? true : false;
  if (flag) {
    if (fabs(delta_euler[2]) > 10.0) {
      HJ_ERROR("function: %s", __FUNCTION__);
      HJ_ERROR("computeDeltaRotation: delta euler is too large, euler angle is %lf, %lf", last_euler[2], curr_euler[2]);
      HJ_ERROR("delta timestamp diff is too large, timestamp is %ld, %ld", imu_data_0.time, imu_data_1.time);
    }
    return delta_euler;
  }
  else {
    if (delta_euler[2] > 300.0) {
      double sign = -1.0;
      delta_euler[2] = sign * (360.0 - delta_euler[2]);
    }
    if (delta_euler[2] < -300.0) {
      double sign = 1.0;
      delta_euler[2] = sign * (360.0 + delta_euler[2]);
    }

    if (fabs(delta_euler[2]) > 10.0) {
      HJ_ERROR("function: %s", __FUNCTION__);
      HJ_ERROR("delta euler is too large, euler angle is %lf, %lf", last_euler[2], curr_euler[2]);
      HJ_ERROR("delta timestamp diff is too large, timestamp is %ld, %ld", imu_data_0.time, imu_data_1.time);
    }
    return delta_euler;
  }
}

Eigen::Vector3d ErrorStateKalmanFilter::computeSocDeltaRotation(const SocImuData &imu_data_0, const SocImuData &imu_data_1) {

  const double last_imu_time_ = static_cast<double>(imu_data_0.time) * 1e-6;
  const double curr_imu_time_ = static_cast<double>(imu_data_1.time) * 1e-6;
  const double delta_t = curr_imu_time_ - last_imu_time_;
//  if (delta_t < 1e-4) {
//    HJ_ERROR("#Soc IMU time error: %lf, %lf", curr_imu_time_, last_imu_time_);
//    delta_t = 0.0;
//  }

  const Eigen::Vector3d &unbias_gyro_0 = computeUnbiasGyro(imu_data_0.angular_velocity_);
  const Eigen::Vector3d &unbias_gyro_1 = computeUnbiasGyro(imu_data_1.angular_velocity_);
  Eigen::Vector3d delta_theta = 0.5 * (unbias_gyro_0 + unbias_gyro_1) * delta_t;
  return delta_theta;
}



void ErrorStateKalmanFilter::computeOrientation(const Eigen::Vector3d &angular_delta,
                                                const Eigen::Matrix3d &R_nm_nm_1,
                                                Eigen::Matrix3d &curr_R,
                                                Eigen::Matrix3d &last_R) {
  Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized());

  last_R = pose_.block<3, 3>(0, 0);
  curr_R = R_nm_nm_1.transpose() * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();
  pose_.block<3, 3>(0, 0) = curr_R;
}

void ErrorStateKalmanFilter::computeOrientationEuler(Eigen::Matrix3d &curr_R,
                                                     Eigen::Matrix3d &last_R, 
                                                     Eigen::Vector3d delta_rotation) {

  last_R = pose_.block<3, 3>(0, 0);
  Eigen::Vector3d curr_euler_angle_ = curr_imu_data_.euler_angle_ * kDegree2Radian;

  double curr_theta = calDeltaThetaBetweenInitAndCurrent(init_imu_theta_[2], curr_imu_data_.euler_angle_[2]);
  //todo
  Eigen::Vector3d tmp_curr_theta = Eigen::Vector3d(curr_euler_angle_[0], curr_euler_angle_[1], curr_theta * kDegree2Radian);
  Eigen::Vector3d tmp_mag_bias_theta = Eigen::Vector3d(0.0, 0.0, init_mag_theta_ * kDegree2Radian);
  Eigen::Vector3d curr_theta_corrected = tmp_curr_theta + icp_diff_theta_ + relocalization_diff_angle_ + tmp_mag_bias_theta;
  
  for (int i = 0; i < 3; i++) {
    while (curr_theta_corrected[i] > M_PI) {
        curr_theta_corrected[i] -= 2 * M_PI;
    }
    while (curr_theta_corrected[i] < -M_PI) {
        curr_theta_corrected[i] += 2 * M_PI;
    }
  }

  auto curr_quaternion = Eigen::AngleAxisd(curr_theta_corrected[2], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(curr_theta_corrected[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(curr_theta_corrected[0], Eigen::Vector3d::UnitX());
//  HJ_INFO("current theta corrected: %lf, %lf, %lf", curr_theta_corrected[0] * kRadian2Degree, curr_theta_corrected[1] * kRadian2Degree, curr_theta_corrected[2] * kRadian2Degree);
  curr_quaternion.normalize();
  curr_R = curr_quaternion.toRotationMatrix();
  pose_.block<3, 3>(0, 0) = curr_R;

}

void ErrorStateKalmanFilter::computeVelocity(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1,
                                             const ImuData &imu_data_0, const ImuData &imu_data_1,
                                             Eigen::Vector3d &last_vel, Eigen::Vector3d &curr_vel) {

  const double last_imu_time_ = static_cast<double>(imu_data_0.time) * 1e-6;
  const double curr_imu_time_ = static_cast<double>(imu_data_1.time) * 1e-6;

//  double delta_t = std::floor((curr_imu_time_ - last_imu_time_) * 1e3) / 1e3;
  double delta_t = curr_imu_time_ - last_imu_time_;

  if (delta_t < 1e-6) {
    hj_interface::HealthCheckCode srv_msg;
    srv_msg.request.code_val = IMU_CHECK_ERROR;
    srv_msg.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
//    hj_bf::HjPushSrv(srv_msg);
    HJ_ERROR("#IMU time error2: %lf, %lf", curr_imu_time_, last_imu_time_);
    delta_t = 0.0;
  }

  Eigen::Vector3d unbias_accel_0 = R_0 * computeUnbiasAccel(imu_data_0.linear_acceleration_)- g_;
  Eigen::Vector3d unbias_accel_1 = R_1 * computeUnbiasAccel(imu_data_1.linear_acceleration_)- g_;

  last_vel = velocity_;

  // 中值积分
  velocity_ += delta_t * 0.5 * (unbias_accel_0 + unbias_accel_1);

  curr_vel = velocity_;
}

Eigen::Vector3d ErrorStateKalmanFilter::computeUnbiasAccel(const Eigen::Vector3d &accel) {
  return accel - accel_bias_;
}

Eigen::Vector3d ErrorStateKalmanFilter::computeUnbiasGyro(const Eigen::Vector3d &gyro) {
  return gyro - gyro_bias_;
}

void ErrorStateKalmanFilter::computePosition(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1,
                                             const Eigen::Vector3d &last_vel, const Eigen::Vector3d &curr_vel,
                                             const ImuData &imu_data_0,
                                             const ImuData &imu_data_1) {

  const double last_imu_time_ = static_cast<double>(imu_data_0.time) * 1e-6;
  const double curr_imu_time_ = static_cast<double>(imu_data_1.time) * 1e-6;
  double delta_t = curr_imu_time_ - last_imu_time_;
  Eigen::Vector3d unbias_accel_0 = R_0 * computeUnbiasAccel(imu_data_0.linear_acceleration_) - g_;
  Eigen::Vector3d unbias_accel_1 = R_1 * computeUnbiasAccel(imu_data_1.linear_acceleration_)- g_;
  pose_.block<3, 1>(0, 3) += 0.5 * delta_t * (curr_vel + last_vel) +
                             0.25 * (unbias_accel_0 + unbias_accel_1) * delta_t * delta_t;

}

void ErrorStateKalmanFilter::resetState() {
  X_.setZero();
}

void ErrorStateKalmanFilter::eliminateError() {
  pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) + X_.block<3, 1>(INDEX_STATE_POSI, 0);

  velocity_ = velocity_ + X_.block<3, 1>(INDEX_STATE_VEL, 0);

  Eigen::Matrix3d C_nn = SO3Exp(-X_.block<3, 1>(INDEX_STATE_ORI, 0));
  pose_.block<3, 3>(0, 0) = C_nn * pose_.block<3, 3>(0, 0);
  last_pose_ = pose_;
  gyro_bias_ = gyro_bias_ + X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
  accel_bias_ = accel_bias_ + X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
}

Eigen::Matrix4d ErrorStateKalmanFilter::getPose() const {
  return pose_;
}


TypeVector1 ErrorStateKalmanFilter::correctYawByMag(TypeVector1 pre_, TypeVector1 measure_) {
  auto yaw_ = yaw_kf_->predictAndupdate(pre_, measure_);
  return yaw_;

}

float ErrorStateKalmanFilter::doMagFusion(MagData &mag, Eigen::Vector3d &angle) {

  MagData curr_encoder_mag = mag;
  curr_encoder_mag.calculateMagDegree(mag.mag_x, mag.mag_y,
                                      tracking_config_parameters_.magnetic_offset_x_, tracking_config_parameters_.magnetic_offset_y_);
  curr_mag_theta_ = curr_encoder_mag.yaw;
//  printf("#mag_result: %f, %ld\n", curr_mag_theta_, curr_encoder_data_.time);

  delta_mag_theta_ = (-1) * (curr_mag_theta_ - init_mag_theta_);
  if (delta_mag_theta_ < 0) {
    delta_mag_theta_ += 360.0;
  }

  curr_fusion_theta_ = angle[0] * kRadian2Degree;
  if (curr_fusion_theta_ < 0) {
    delta_fusion_theta_ = 360.0 + curr_fusion_theta_;
  }
  else {
    delta_fusion_theta_ = curr_fusion_theta_;
  }
  TypeVector1 pre_;
  TypeVector1 measure_;
  TypeVector1 yaw_matrix;
  pre_ << delta_fusion_theta_;
  measure_ << delta_mag_theta_;
//  printf("#mag_bias_result: %f, %f, %ld\n", pre_(0,0), measure_(0, 0), curr_encoder_data_.time);
  if (fabs(measure_(0, 0) - pre_(0,0)) < 20.0) {
    yaw_matrix = correctYawByMag(pre_, measure_);
  } else {
    if (delta_fusion_theta_ > 180.0) {
      delta_fusion_theta_ = -(360.0 - delta_fusion_theta_);
    }
    return delta_fusion_theta_;
  }

  float result{0.0f};
  if (yaw_matrix(0,0) > 180.0) {
    result = -(360.0 - yaw_matrix(0,0));
  }
  else {
    result = yaw_matrix(0,0);
  }
  return result;

}

void ErrorStateKalmanFilter::doDepthFusion(float &depth_z) {
  pose_(2, 3) = depth_z;
}


bool ErrorStateKalmanFilter::correctByUltra(UltraData &ultra_measurement, bool &get_ultra, Eigen::Vector3d &angle, Eigen::Vector3d &position) {
  bool correct_by_ultra = false;
  if (get_ultra && ultra_measurement.front_m_ < 60000) {
    Eigen::Matrix<double, 8, 1> angle_pos;
    double yaw_switch = angle[0] * kRadian2Degree < 0 ? angle[0] * kRadian2Degree + 360.0 : angle[0] * kRadian2Degree;
    double time = static_cast<double>(currEncoder->getTime()) * 1e-6;
    double ultra = static_cast<double>(ultra_measurement.front_m_) / 1000.0;

    angle_pos << time, yaw_switch, angle[1] * kRadian2Degree, angle[2] * kRadian2Degree, position[0], position[1], position[2], ultra;
    time_angle_pose_front_ultra.push_front(angle_pos);
    while (time_angle_pose_front_ultra.size() > 50) {
      time_angle_pose_front_ultra.pop_back();
    }
    correct_by_ultra = true;
  }
  else{

    Eigen::Matrix<double, 8, 1> angle_pos;
    double time_ = static_cast<double>(currEncoder->getTime()) * 1e-6;
    double ultra_ = 0.0;
    double yaw_switch = 0.0;
    if (angle[0] * kRadian2Degree < 0) {
      yaw_switch = angle[0] * kRadian2Degree + 360.0;
    }
    else {
      yaw_switch = angle[0] * kRadian2Degree;
    }

    angle_pos << time_, yaw_switch, angle[1] * kRadian2Degree, angle[2] * kRadian2Degree,
            position[0], position[1], position[2], ultra_;
    time_angle_pose_front_ultra.push_front(angle_pos);
    while (time_angle_pose_front_ultra.size() > 50) {
      time_angle_pose_front_ultra.pop_back();
    }
    correct_by_ultra = false;
  }

  bool is_corrected = false;
  if (correct_by_ultra) {
    is_corrected = correctByUltraData();
    return is_corrected;
  }

  return is_corrected;

}

void ErrorStateKalmanFilter::putICPTimestamp(const uint64_t curr_time) {
  std::unique_lock<std::mutex> lck(icp_carrier_lock_);
  circle_timestamp_buffer_.push_back(curr_time);
}

void ErrorStateKalmanFilter::setRelocalizationSuccess() {
  is_relocalization_success_ = true;
}

void ErrorStateKalmanFilter::setRelocalizationPose(const Eigen::Vector3d &position, const Eigen::Vector3d &euler_angle_diff) {
  relocalization_position_ = position;
  relocalization_diff_angle_ += euler_angle_diff;
  for (int i = 0; i < 3; i++) {
    while (relocalization_diff_angle_[i] > M_PI) {
        relocalization_diff_angle_[i] -= 2 * M_PI;
    }
    while (relocalization_diff_angle_[i] < -M_PI) {
        relocalization_diff_angle_[i] += 2 * M_PI;
    }
  }
}

bool ErrorStateKalmanFilter::getICPTimestamp(std::vector<uint64_t> &time_carriers) {
  std::unique_lock<std::mutex> lck(icp_carrier_lock_);
  if (circle_timestamp_buffer_.empty()) {
    return false;
  }
  while (!circle_timestamp_buffer_.empty()) {
    auto time = circle_timestamp_buffer_.front();
    circle_timestamp_buffer_.pop_front();
    time_carriers.push_back(time);
  }
  return true;
}

static bool compareByMagX(const Eigen::Matrix<double, 7, 1>& a, const Eigen::Matrix<double, 7, 1>& b) {
  return a(4) > b(4);
}

static bool compareByMagY(const Eigen::Matrix<double, 7, 1>& a, const Eigen::Matrix<double, 7, 1>& b) {
  return a(5) > b(5);
}

void ErrorStateKalmanFilter::setMagOffset(Eigen::Vector3d &magOffset) {
  mag_offset_[0] = magOffset[0];
  mag_offset_[1] = magOffset[1];
}

Eigen::Vector3d ErrorStateKalmanFilter::calMagOffset() {
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Tw_mag;
  Tw_mag << 0, -1, 0, -1, 0, 0, 0, 0, -1;
  for (std::list<Eigen::Matrix<double, 7, 1>>::iterator it = time_euler_mag.begin(); it != time_euler_mag.end(); ++it) {
    Eigen::Quaterniond quaternion = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd((*it)[2], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd((*it)[1], Eigen::Vector3d::UnitX());
    quaternion.normalize();
    Eigen::Vector3d mag_b((*it)[4], (*it)[5], (*it)[6]);
    Eigen::Vector3d mag_w = quaternion.toRotationMatrix() * Tw_mag * mag_b;
    (*it)[4] = mag_w[0];
    (*it)[5] = mag_w[1];
    (*it)[6] = mag_w[2];
  }
  double x_mag_max = std::numeric_limits<double>::lowest();
  double x_mag_min = std::numeric_limits<double>::max();
  double y_mag_max = std::numeric_limits<double>::lowest();
  double y_mag_min = std::numeric_limits<double>::max();
  for (std::list<Eigen::Matrix<double, 7, 1>>::iterator it = time_euler_mag.begin(); it != time_euler_mag.end(); ++it) {
    double x = (*it)[4];
    double y = (*it)[5];
    if (x > x_mag_max) {
      x_mag_max = x;
    }
    if (x < x_mag_min) {
      x_mag_min = x;
    }
    if (y > y_mag_max) {
      y_mag_max = y;
    }
    if (y < y_mag_min) {
      y_mag_min = y;
    }
  }

  result[0] = (x_mag_max - x_mag_min) / 2 + x_mag_min;
  result[1] = (y_mag_max - y_mag_min) / 2 + y_mag_min;
  if (y_mag_max - y_mag_min < 1e-3) {
    result[2] = 0.0;
  }
  else {
    result[2] = (x_mag_max - x_mag_min) / (y_mag_max - y_mag_min);
  }
  return result;

}

bool ErrorStateKalmanFilter::setInitialPose(EncoderDataPtr &curr_encoder, ImuMeasurements &imus,
                                            MagData &mag, bool &is_mag, bool &is_mag_success_init) {
  if (!is_mag) {
    HJ_INFO("init is failure because of no mag data");
    return false;
  }
  Eigen::Vector3d acc_ = Eigen::Vector3d(accel_bias_[0], accel_bias_[1], accel_bias_[2] + 9.80);
  Eigen::Vector3d gyro_ = Eigen::Vector3d(gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
  curr_encoder->interpolationAccGyroEuler(curr_encoder->getTime(), imus);
  init_imu_theta_ = curr_encoder->getEuler();
  HJ_INFO("init rotation by imu (uint:degree) = %f, %f, %f", init_imu_theta_[0], init_imu_theta_[1], init_imu_theta_[2]);
  Eigen::Vector3d init_angular = curr_encoder->getEuler() * kDegree2Radian;
  init_angular[2] = 0;
  Eigen::Quaterniond init_quaternion = Eigen::AngleAxisd(init_angular[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(init_angular[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(init_angular[0], Eigen::Vector3d::UnitX());
  init_quaternion.normalize();
  Eigen::Vector3d init_imu_ = Eigen::Vector3d(curr_encoder->getAcc()[0], curr_encoder->getAcc()[1], curr_encoder->getAcc()[2]);
  Eigen::Vector3d bias_ = init_imu_ - (init_quaternion.toRotationMatrix().inverse()) * Eigen::Vector3d(0, 0, 9.8);
  acc_ = Eigen::Vector3d(bias_[0], bias_[1], bias_[2] + 9.80);
  accel_bias_ = bias_;

// init mag
if (is_mag_success_init) {
  Eigen::Matrix3d Tw_mag;
  Tw_mag << 0, -1, 0, -1, 0, 0, 0, 0, -1;
  Eigen::Matrix<double, 3, 1> mag_mat;
  mag_mat << mag.mag_x, mag.mag_y, mag.mag_z;
  Eigen::Vector3d mag_check = init_quaternion.toRotationMatrix() * Tw_mag * mag_mat;
  HJ_INFO("mag offset is %f, %f", mag_offset_[0], mag_offset_[1]);
  float result_yaw = -1 * mag.calculateMagDegree(mag_check[0], mag_check[1], mag_offset_[0], mag_offset_[1]);
  Eigen::Quaterniond yaw_rotation =
          Eigen::AngleAxisd(static_cast<double>(result_yaw) * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(init_angular[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(init_angular[0], Eigen::Vector3d::UnitX());
  HJ_INFO("init rotation by mag (uint:degree) = %f, %f, %f", init_angular[0] * kRadian2Degree, init_angular[1] * kRadian2Degree,
          static_cast<double>(result_yaw));
  init_mag_theta_ = result_yaw;
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) * yaw_rotation.toRotationMatrix();
}
else {
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) * init_quaternion.toRotationMatrix();
}
//todo
/*
//  Eigen::Matrix3d R_imu_to_ned;
//  R_imu_to_ned << 0,  1,  0,
//          1,  0,  0,
//          0,  0, -1;
//  // 组合旋转：从 IMU 坐标系到 NED 坐标系
//  q_imu_to_ned_ = Eigen::Quaterniond(R_imu_to_ned) * yaw_rotation;
//  Eigen::Quaterniond q_imu = quaternion;
//  Eigen::Vector3d t_imu = Eigen::Vector3d::Zero();
//  Eigen::Quaterniond q_ned = q_imu_to_ned_ * q_imu; // 转换旋转
//  Eigen::Vector3d t_ned = R_imu_to_ned * t_imu;    // 转换平移
//  pose_.block<3, 3>(0, 0) = q_ned.toRotationMatrix();
//  pose_.block<3, 1>(0, 3) = t_ned;
//
//  Eigen::Vector3d init_angle = Quaternion2EulerAngles(q_ned);
//  HJ_INFO("roll = %f", init_angular[0] * kRadian2Degree);
//  HJ_INFO("pitch = %f", init_angular[1] * kRadian2Degree);
//  HJ_INFO("init rotation = %f, %f, %f", init_angle[2] * kRadian2Degree, init_angle[1] * kRadian2Degree, init_angle[0] * kRadian2Degree);
//  HJ_INFO("init translation = %f, %f, %f", t_ned[0], t_ned[1], t_ned[2]);
 */

  ImuData m_(curr_encoder->getTime(), gyro_, acc_, init_imu_theta_);
  last_imu_data_ = m_;

  return true;

}

void ErrorStateKalmanFilter::doPredicting(ImuMeasurements &imus) {
  for (int i = imus.size() - 1; i >= 0; --i) {
    if (imus[i].time > lastEncoder->getTime() && imus[i].time < currEncoder->getTime()) {
      predict(imus[i]);
    }
  }
  //  //continue poprogation
  currEncoder->interpolationAccGyroEuler(currEncoder->getTime(), imus);

  ImuData curr_encoder_imu = ImuData(currEncoder->getTime(), currEncoder->getGyro(), currEncoder->getAcc(), currEncoder->getEuler());
  predict(curr_encoder_imu);

  last_imu_data_ = curr_encoder_imu;

}

#ifdef Aiper_surface_vessel
Eigen::Vector3d ErrorStateKalmanFilter::doPredicting(SocImuMeasurements &imus) {
  Eigen::Vector3d delta_theta_result = Eigen::Vector3d::Zero();
  if (imus.size() == 0)
    return delta_theta_result;

  for (int i = imus.size() - 1; i >= 0; --i) {
    if (imus[i].time > lastEncoder->getTime() && imus[i].time < currEncoder->getTime()) {
      double dt = (imus[i].time - lastEncoder->getTime()) * 1e-6;

      Eigen::Vector3d delta_theta = predict(imus[i]);
      Eigen::Vector3d acc = Eigen::Vector3d(imus[i].linear_acceleration_[0], imus[i].linear_acceleration_[1], imus[i].linear_acceleration_[2]);
      double roll_acc = atan2(acc[1], acc[2]);
      double pitch_acc = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2]));
//      roll = alpha * (roll + delta_theta[0]) + (1 - alpha) * roll_acc;
//      pitch = alpha * (pitch + delta_theta[1]) + (1 - alpha) * pitch_acc;

      delta_theta_result += delta_theta;

    }
  }
  //  //continue poprogation
  SocImuData curr_encoder_Acc_Gyro = interpolationAcc(currEncoder->getTime(), imus);

  Eigen::Vector3d delta_theta = predict(curr_encoder_Acc_Gyro);

  last_soc_imu_data_ = curr_encoder_Acc_Gyro;

  delta_theta_result += delta_theta;
  return delta_theta_result;
}
#endif

double ErrorStateKalmanFilter::calDeltaThetaBetweenInitAndCurrent(const double origin_theta, const double curr_theta) {
  bool flag = origin_theta * curr_theta >= 0.0 ? true : false;

  if (flag) {
    return curr_theta - origin_theta;
  }
  else {
    double temp_left = 180.0 - fabs(origin_theta);
    double temp_right = 180.0 - fabs(curr_theta);
    double sign = curr_theta > 0.0 ? -1.0 : 1.0;
    return sign * (temp_left + temp_right);
  }

}


bool ErrorStateKalmanFilter::correctByUltraData() {

  std::list<double> yaw_list;
  std::list<double> pitch_list;
  std::list<double> roll_list;
  for (auto it = time_angle_pose_front_ultra.begin(); it != time_angle_pose_front_ultra.end(); ++it) {
    yaw_list.push_back((*it)[1]);//yaw
    pitch_list.push_back((*it)[2]);//pitch
    roll_list.push_back((*it)[3]);//roll
  }
  auto y_min = min_element(yaw_list.begin(), yaw_list.end());
  auto y_max = max_element(yaw_list.begin(), yaw_list.end());
  double yaw_min = *y_min;
  double yaw_max = *y_max;
//  auto p_min = min_element(pitch_list.begin(), pitch_list.end());
//  auto p_max = max_element(pitch_list.begin(), pitch_list.end());
//  double pitch_min = *p_min;
//  double pitch_max = *p_max;
//  auto r_min = min_element(roll_list.begin(), roll_list.end());
//  auto r_max = max_element(roll_list.begin(), roll_list.end());
//  double roll_min = *r_min;
//  double roll_max = *r_max;
//
  //first step : judge yaw
  bool judge_yaw = fabs(yaw_max - yaw_min) < 5.0;

  //choose the most similiar yaw
  double yaw_begin = time_angle_pose_front_ultra.back()[1];
  double min_yaw = 1000.0;
  Eigen::Matrix<double, 8, 1> most_suitable_data;
  Eigen::Matrix<double, 8, 1> data_begin;
  int begin_index = 0;
  for (auto it = time_angle_pose_front_ultra.begin(); it != time_angle_pose_front_ultra.end(); ++it, ++begin_index) {
    if ((*it)[7] == 0.0) {
      continue;
    }
    data_begin = (*it);
    break;
  }

  int end_index = 0;
  for (auto it = time_angle_pose_front_ultra.begin(); it != time_angle_pose_front_ultra.end(); ++it, ++end_index) {
    if (end_index <= begin_index) {
      continue;
    }
    double yaw_curr = (*it)[1];
    double diff_yaw = fabs(yaw_curr - data_begin[1]);
    if (diff_yaw < min_yaw) {
      most_suitable_data = (*it);
      min_yaw = diff_yaw;
    }
  }
  bool judge_pitch_suitable = fabs(data_begin[2] - most_suitable_data[2]) < 1.0;
  bool judge_roll_suitable = fabs(data_begin[3] - most_suitable_data[3]) < 1.0;
  if (min_yaw > 0.2 || !judge_pitch_suitable || !judge_roll_suitable) {
    time_angle_pose_front_ultra.clear();
    return false;
  }

//  bool judge_pitch =  0.5 * (pitch_max + pitch_min) < 5.0;
//  bool judge_roll =  0.5 * (roll_max + roll_min) < 5.0;
  bool judge_angle = judge_yaw;

  if (judge_angle && time_angle_pose_front_ultra.size() >= 50) {
    double ave_yaw = yaw_begin;
    double valid_ultra_suitable = most_suitable_data[7];
    double curr_pos_x = most_suitable_data[4];
    double curr_pos_y = most_suitable_data[5];

    double valid_ultra_begin = data_begin[7];
    double last_pos_x = data_begin[4];
    double last_pos_y = data_begin[5];

    double valid_delta_ultra = fabs(valid_ultra_suitable - valid_ultra_begin);
    double delta_x = valid_delta_ultra * cos(ave_yaw * kDegree2Radian);
    double delta_y = valid_delta_ultra * sin(ave_yaw * kDegree2Radian);

    double opt_pose_x = last_pos_x + delta_x;
    double opt_pose_y = last_pos_y + delta_y;
    double distance_ = (curr_pos_x - opt_pose_x) * (curr_pos_x - opt_pose_x) +
                       (curr_pos_y - opt_pose_y) * (curr_pos_y - opt_pose_y);

    if (distance_ > 0.15 * 0.15) {
      time_angle_pose_front_ultra.clear();
      return false;
    }

    bool is_pos_x_enough = fabs(curr_pos_x - last_pos_x) > 0.03;
    bool is_pos_y_enough = fabs(curr_pos_y - last_pos_y) > 0.03;
    if (!is_pos_x_enough && !is_pos_y_enough) {
      time_angle_pose_front_ultra.clear();
      return false;
    }

    pose_(0,3) += delta_x;
    pose_(1,3) += delta_y;


    time_angle_pose_front_ultra.clear();
    optimization_count++;
    return true;
  }
  else if(judge_angle && time_angle_pose_front_ultra.size() < 50) {
    return false;
  //todo
  }
  else {
    //Eigen::Matrix<double, 8, 1> first_ = time_angle_pose_front_ultra.front();
    time_angle_pose_front_ultra.clear();
    //time_angle_pose_front_ultra.push_front(first_);
    return false;
  }

}

ErrorStateKalmanFilter::~ErrorStateKalmanFilter() {
#ifdef Save_File
  ofs_pose.close();
#endif
}
} // end namespace HJ_tracking
} // end namespace HJ_slam