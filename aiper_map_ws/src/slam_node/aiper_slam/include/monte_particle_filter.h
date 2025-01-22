#pragma once

#include "imu_data.h"
#include "soc_imu_data.h"
#include "encoder_data.h"
#include "mag_data.h"
#include "ultra_data.h"
#include "pressure_data.h"
#include "pose_data.h"
#include "mag_output.h"
#include "mag_ekf.h"
#include "point.h"
#include "config_parameters.h"
#include "status_code.h"
#include "node_cache.h"
#include "common_tool.h"

#include "log.h"
#include <eigen3/Eigen/Dense>
#include <deque>
#include <list>
#include <thread>
#include <condition_variable>
#include <unordered_map>
#include <mutex>
// #define PI 3.1416
//#define Save_File
class Yaw_Kalman_Filter;
typedef std::shared_ptr<Yaw_Kalman_Filter> Yaw_Kalman_FilterPtr;

//HJ_slam::HJ_mapping::BuildMapWithTwoSonarPtr build_map_ptr_;
//class BuildMapWithTwoSonar;
//std::shared_ptr<BuildMapWithTwoSonar> BuildMapWithTwoSonar_ptr_;

namespace HJ_slam {
namespace HJ_tracking {

class ErrorStateKalmanFilter {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit ErrorStateKalmanFilter(const TrackingConfigParameters &config_parameters);
  ~ErrorStateKalmanFilter();

 /*!
  * 实现滤波器的reset功能，需要重新初始化
  */
  void softReset();

 /*!
  * 实现滤波，对imu的异常值进行滤波
  */
  void filterImuData(ImuMeasurements &imus);

 /*!
  * IMU姿态解算，以及滤波器的预测步的构建，对应卡尔曼滤波器的前两个公式
  * @param curr_imu_data mcu imu
  * @return 旋转角的增量
  */
  void predict(const ImuData &curr_imu_data);

 /*!
  * IMU姿态解算，以及滤波器的预测步的构建，对应卡尔曼滤波器的前两个公式
  * @param curr_imu_data soc imu
  * @return 旋转角的增量
  */
  Eigen::Vector3d predict(const SocImuData &curr_imu_data);

 /*!
  * Initialize SLAM, determine initial pose, set machine initialization state.
  */
  bool initialize(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, MagData &mag, bool &is_mag, bool &is_mag_success_init);

 /*!
  * 滤波主程序，对齐之后的玛盘和imu数据
  * @param curr_encoder_data， ImuMeasurements &imus， MagData &mag
  * @return
  */
  PoseDataPtr process(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus, 
                      MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra);

  MagOutputDataPtr estimateMagYaw(EncoderDataPtr &curr_encoder, MagData &mag, bool &is_mag);
 /*!
  * 滤波程序，输出3dof的位姿
  * @param curr_encoder_data， ImuMeasurements &imus
  * @return
  */
  PoseDataPtr process_three_dof(EncoderDataPtr &curr_encoder, ImuMeasurements &imus);

 /*!
  * 采集旋转过程中的磁力计数据
  * @param curr_encoder_data， MagData &mag
  * @return
  */
  void collectMagData(EncoderDataPtr &curr_encoder, Eigen::Vector3d &mag, bool &is_mag, bool &collect_ending);

  void setIcpResult(Eigen::Vector3d icp_rotation_result) {
    icp_diff_theta_ = icp_rotation_result;
  }
  //todo
  std::list<Eigen::Matrix<double, 8, 1>>time_angle_pose_front_ultra;

 /*!
  * 初始化
  * @param
  * @return
  */
  bool setInitialPose(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, MagData &mag, bool &is_mag, bool &is_mag_success_init);

 /*!
  * 插值，需要插值出玛盘时刻的accel，gyro
  * @param curr_encoder_data， SocImuMeasurements &imus
  * @return ImuData
  */
  SocImuData interpolationAcc(uint64_t ts, SocImuMeasurements &imus);

 /*!
  * 计算磁力计的标定值 offset
  * @param
  * @return MagOffset
  */
  Eigen::Vector3d calMagOffset();
  void setMagOffset(Eigen::Vector3d &magOffset);
 /*!
  * 滤波器的预测
  * @param ImuMeasurements mcu imus
  * @return
  */
  void doPredicting(ImuMeasurements &imus);

 /*!
  * 滤波器的预测
  * @param SocImuMeasurements soc imus
  * @return
  */
  Eigen::Vector3d doPredicting(SocImuMeasurements &soc_imus);

 /*!
  * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
  * @param curr_gps_data
  * @return
  */
  bool correct();

  void doDepthFusion(float &depth_z);

  Eigen::Matrix4d getPose () const;

  void AlterPose(Eigen::Matrix4d &temp) {
    pose_ = temp;
  }

//  Eigen::Vector3d getVelocity() {
//    return velocity_;
//  }

  bool getResetFlag() {
    return need_reset;
  }

  bool getICPTimestamp(std::vector<uint64_t> &time_carriers);

  void setRelocalizationSuccess();
  void setRelocalizationPose(const Eigen::Vector3d &position, const Eigen::Vector3d &euler_angle_diff);

  Eigen::Quaterniond q_imu_to_ned_;

  std::list<Eigen::Matrix<double, 7, 1>> time_euler_mag;
private:
  void setCovarianceQ(double gyro_noise_cov, double accel_noise_cov);

  /*!
   *
   * @param position_x_std
   */
  void setCovarianceR(double position_x_std, double position_y_std, double position_z_std,
                      double velocity_x_std, double velocity_y_std, double velocity_z_std);

  void setCovarianceP(double posi_noise, double velocity_noise, double ori_noise,
                      double gyro_noise, double accel_noise);

  /*!
   * 通过IMU计算位姿和速度
   * @return
   */
  void updateOdomEstimation(const Eigen::Vector3d &w_in, Eigen::Vector3d &delta_rotation);

#ifdef Aiper_surface_vessel
  void updateSocRotation(const Eigen::Vector3d &w_in, Eigen::Vector3d &delta_rotation);
#endif

  void updateErrorState(double t, const Eigen::Vector3d &accel, const Eigen::Vector3d &w_in_n);

  Eigen::Vector3d computeDeltaRotation(const ImuData &imu_data_0, const ImuData &imu_data_1);

  Eigen::Vector3d computeSocDeltaRotation(const SocImuData &imu_data_0, const SocImuData &imu_data_1);

  /*!
   * 计算地球转动给导航系带来的变换
   * note: 对于普通消费级IMU可以忽略此项
   * @param R_nm_nm_1
   * @return
   */
  //Eigen::Vector3d computeNavigationFrameAngularVelocity();

  /*!
   * 通过IMU计算当前姿态
   * @param angular_delta
   * @param R_nm_nm_1
   * @param curr_R
   * @param last_R
   * @return
   */
  void computeOrientation(const Eigen::Vector3d &angular_delta,
                          const Eigen::Matrix3d &R_nm_nm_1,
                          Eigen::Matrix3d &curr_R,
                          Eigen::Matrix3d &last_R);

  /*!
   * 通过Euler计算当前姿态
   * @param curr_R
   * @param last_R
   * @return
   */
  void computeOrientationEuler(Eigen::Matrix3d &curr_R, Eigen::Matrix3d &last_R, Eigen::Vector3d delta_rotation);

  void computeVelocity(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1, const ImuData &imu_data_0,
                       const ImuData &imu_data_1, Eigen::Vector3d &last_vel, Eigen::Vector3d &curr_vel);

  Eigen::Vector3d computeUnbiasAccel(const Eigen::Vector3d &accel);

  Eigen::Vector3d computeUnbiasGyro(const Eigen::Vector3d &gyro);

  /*!
   * 通过imu计算当前位移
   * @param curr_vel
   * @param last_vel
   * @return
   */
  void computePosition(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1, const Eigen::Vector3d &last_vel, const Eigen::Vector3d &curr_vel, const ImuData &imu_data_0,
                       const ImuData &imu_data_1);

  /*!
   * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
   */
  void eliminateError();

  /*!
   * 每次矫正之后，需要重置状态变量X
   */
  void resetState();

  /*!
 * 每次矫正之后，需要重置滤波状态的所有变量
 */
  void resetAllState();

  double calDeltaThetaBetweenInitAndCurrent(const double origin_theta, const double curr_theta);

  bool correctByUltraData();

  bool doMagInitial(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, MagData &mag, bool &is_mag);

  float doMagFusion(MagData &mag, Eigen::Vector3d &angle);

  bool correctByUltra(UltraData &ultra_measurement, bool &get_ultra, Eigen::Vector3d &angle, Eigen::Vector3d &position);

  TypeVector1 correctYawByMag(TypeVector1 pre_, TypeVector1 measure_);

  void putICPTimestamp(const uint64_t curr_time);

private:
  static const unsigned int DIM_STATE = 15;
  static const unsigned int DIM_STATE_NOISE = 6;
  static const unsigned int DIM_MEASUREMENT = 6;
  static const unsigned int DIM_MEASUREMENT_NOISE = 6;

  static const unsigned int INDEX_STATE_POSI = 0;
  static const unsigned int INDEX_STATE_VEL = 3;
  static const unsigned int INDEX_STATE_ORI = 6;
  static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
  static const unsigned int INDEX_STATE_ACC_BIAS = 12;
  static const unsigned int INDEX_MEASUREMENT_POSI = 0;
  static const unsigned int INDEX_MEASUREMENT_VEL = 3;

  typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;
  typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
  typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

  TypeVectorX X_ = TypeVectorX::Zero();
  TypeVectorY Y_ = TypeVectorY::Zero();
  TypeMatrixF F_ = TypeMatrixF::Zero();
  TypeMatrixB B_ = TypeMatrixB::Zero();
  TypeMatrixQ Q_ = TypeMatrixQ::Zero();
  TypeMatrixP P_ = TypeMatrixP::Zero();
  TypeMatrixK K_ = TypeMatrixK::Zero();
  TypeMatrixC C_ = TypeMatrixC::Zero();
  TypeMatrixG G_ = TypeMatrixG::Zero();
  TypeMatrixC R_ = TypeMatrixR::Zero();
  TypeMatrixF Ft_ = TypeMatrixF::Zero();

  //config
  TrackingConfigParameters tracking_config_parameters_;

  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

  //external axis imu
  Eigen::Matrix3d Twb_mcu_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Twb_euler_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Twb_soc_ = Eigen::Matrix3d::Zero();
  Eigen::Vector3d t_bm_x9 = Eigen::Vector3d(-0.11, 0.0, 0.0);
  Eigen::Vector3d t_bm_t1_pro = Eigen::Vector3d(0.0, 0.0, 0.0);

  //imu bias
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  //encoder
  EncoderDataPtr currEncoder = nullptr;
  EncoderDataPtr lastEncoder = nullptr;

  //imu
  Eigen::Vector3d init_imu_theta_ = Eigen::Vector3d::Zero();
  ImuData last_imu_data_;
  ImuData curr_imu_data_;

  //soc_imu
#ifdef Aiper_surface_vessel
  SocImuData last_soc_imu_data_;
  SocImuData curr_soc_imu_data_;
  Eigen::Vector3d angle_soc_imu = Eigen::Vector3d::Zero();
#endif

  //init
  bool is_first_mag = true;
  double init_yaw_sum = 0.0;
  Eigen::Vector3d mag_offset_ = Eigen::Vector3d::Zero();

  //optimize
  int optimization_count = 0;
  bool need_reset = false;

  Eigen::Matrix4d last_pose_ = Eigen::Matrix4d::Identity();

  //mag fusion
  Yaw_Kalman_FilterPtr yaw_kf_ = nullptr;

  float init_mag_theta_ = 0.0;
  float curr_mag_theta_ = 0.0;
  float delta_mag_theta_ = 0.0;
  float curr_fusion_theta_ = 0.0;
  float delta_fusion_theta_ = 0.0;

  //correct and get robot state

  std::list<Eigen::Matrix<double, 6, 1>>angle_pose;

  //icp
  Eigen::Vector3d icp_diff_theta_ = Eigen::Vector3d::Zero();

  // corrected by relocalization
  bool is_relocalization_success_ = false;
  Eigen::Vector3d relocalization_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d relocalization_diff_angle_ = Eigen::Vector3d::Zero();

  void checkImuAxis(ImuMeasurements &imus);
  void checkSocImuAxis(SocImuMeasurements &soc_imus);

#ifdef Save_File
    std::ofstream ofs_pose;
#endif

public:
  void getFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y);
};
} // namespace HJ_tracking
} // namespace HJ_slam