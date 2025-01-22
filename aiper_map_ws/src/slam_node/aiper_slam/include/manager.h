#pragma once

#include <queue>
#include <condition_variable>
#include <thread>
#include <ros/ros.h>
#include <future>

#include "config_parameters.h"
#include "interface.h"
#include "imu_data.h"
#include "soc_imu_data.h"
#include "encoder_data.h"
#include "pressure_data.h"
#include "mag_data.h"
#include "ultra_data.h"
#include "navi_slam_data.h"
#include "pose_data.h"
#include "monte_particle_filter.h"
#include "map_modules/buildMapWithTwoSonar.h"
#include "map_modules/mapPoint.h"
#include "left_front_ultra_data.h"
#include "left_back_ultra_data.h" 
#include "relocalization/relocalization.h"
#include "hj_interface/AlgWorkResult.h"
#include "hj_interface/SensorDataRecord.h"

class SLAMManager {
 public:
  enum Stage {
    STAGE_TRACKING = 0,
    STAGE_TRACKING_RELOCALIZATION = 1,
    STAGE_TRACKING_MAPPING = 2
    // STAGE_RESET = 3 // TODO(edan): implement resetting
  };
  SLAMManager(const std::string &config_file_path, const std::string &output_directory,
              const std::string &relocalization_directory, const std::string &vocabulary);
  ~SLAMManager();

  void SoftReset();
  void SoftSensorReset();
  void PutHmdRelocalizationData(const Eigen::Matrix4d &relocalization_pose, const Eigen::Matrix4d &water_entry_pose, 
                                uint8_t relocalization_result, uint8_t build_frames_result, const uint8_t &curr_task);
  void PutHmdTaskData(const uint8_t &curr_task);
  void PutHmdImuData(const IMUData &imu_data);
  void PutHmdSocImuData(const SOCIMUData &soc_imu_data);
  void PutHmdMagData(const MAGData &mag_data);
  void PutHmdUltraData(const ULtraData &ultra_data);
  void PutHmdDepthData(const DEPData &dep_data);
  void PutHmdLeftFrontUltraData(const LEFTFrontData &left_front_data);
  void PutHmdLeftBackUltraData(const LEFTBackData &left_back_data);
  void PutHmdLeftTofData(const LEFTTofData &left_tof_data);
  void PutNaviSlamData(const NAVISlamData &navi_slam_data);
  void PutEncoderData(const ENcoderData &data_, const uint64_t timestamp_nanoseconds);
  bool getLowFrequencyPose(uint64_t ts);
  PoseDataPtr getPose();
  void sendFusionPose(PoseDataPtr &output_pose);
  void sendMagAngle(MagOutputDataPtr &output_mag_angle);
  Stage getStage();
  bool isInitSuccess();
  std::list<MapPointPtr>& getICPSourceCloud();
  std::list<MapPointPtr> getICPTargetCloud();

  bool new_result_flag = false;
  std::queue<PoseDataPtr> output_pose_queue;
  std::mutex output_lock_;
  std::queue<MagOutputDataPtr> output_mag_angle_queue;
  std::mutex output_mag_lock_;
  bool ending = false;

private:
  // SLAM main thread
  void ThreadFunction();
  bool addTask(const uint8_t &task);
  void GetTaskInfo();
  bool addImuMeasurement(const ImuData &m);
  bool addSocImuMeasurement(const SocImuData &m);
  bool addMagMeasurement(const MagData &m);
  bool addPressureMeasurement(const PressureData &p);
  bool addUltraMeasurement(const UltraData &u);
  bool addNaviSlamMeasurement(const NaviSlamData &data);

  bool getMeasurementsContainingEdges(const double frame_timestamp,  // seconds
                                      ImuMeasurements& extracted_measurements,
                                      const bool remove_measurements);

  bool getSocMeasurementsContainingEdges(const double frame_timestamp,  // seconds
                                      SocImuMeasurements& extracted_measurements,
                                      const bool remove_measurements);

  bool getNearestMag(const double frame_timestamp, MagData &mag_measurement);

  bool getNearestUltra(const double frame_timestamp, UltraData &measurement);

  bool getNearestPressure(const double frame_timestamp, PressureData &press_measurement);

  void checkPressureData(bool &get_pressure, PressureData &pressure_measurement);

  bool getTaskID(uint8_t &task_id);

  bool getNaviToSlamInfo(NaviSlamData &data_info);

  void trackingOnceThreeDof(EncoderDataPtr &curr_encoder, ImuMeasurements &imus);

  void collectMagData(EncoderDataPtr &curr_encoder, MagData &mag, bool &is_mag);

  bool computeMagOffset(Eigen::Vector3d &magOffset);

  void setMagOffset(Eigen::Vector3d &magOffset);

  PoseDataPtr trackingOnce(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus,
                             MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra,
                             float &depth_z);

  void relocalizeOnce();

  void mappingOnce(const PoseDataPtr& pose);

  void exportRelocResultToESKF();

  void exportRelocResultToApp(bool success);

  void exportRelocResultToNavi(bool success);

  void exportMapToAPP(const std::list<MapPointPtr> &map_point_list, const Eigen::Vector3f &water_entry_position);

  void exportRecordActionToMiddleware(const uint8_t& task_id);

  void getMapFromAPPTest();

  void waterEntryPointUpdate();

  bool needsRelocalization();

  void resetRelocalization();

  bool isRelocalizationSuccess();

  bool isRelocalizationFailed();

  bool isBuildFramesSuccess();

  bool isBuildFramesFailed();

  bool needsMapping();

  int releaseRelocalization();

  bool isMappingFinished();

  void setStage(const Stage& stage);
  
  void updateState(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus,
                               MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra, 
                               float &depth_z);

  bool GetEncoderData(EncoderDataPtr &encoder_data, uint64_t &ts);

  // get the fusion pose after measurement
  std::shared_ptr<HJ_slam::HJ_tracking::ErrorStateKalmanFilter> eskf_ptr_;

  std::thread thread_;

  std::queue<ENcoderData> encoder_queue;
  TrackingConfigParameters tracking_config_parameters_;

  std::deque<ImuData> measurements_;
  std::deque<SocImuData> soc_measurements_;
  std::deque<MagData> mag_measurements_;
  std::deque<PressureData> pressure_measurements_;
  std::deque<UltraData> ultra_measurements_;
  std::queue<NaviSlamData> navi_slam_measurements_;
  std::deque<LeftFrontUltraData> left_front_ultra_measurements_;
  std::deque<LeftBackUltraData> left_back_ultra_measurements_;
  std::queue<uint8_t> task_lists_;
  int task_id_ = -1;
  bool rotation_has_already_starting_ = false;
  bool rotation_has_already_ending_ = false;
  bool is_mag_init = false;
  bool mag_init_success = false;

  // encoder buffer
  std::mutex encoder_lock_;
  std::condition_variable encoder_condition_;

  // stage
  Stage stage_;
  bool is_init_success_ = false;

  bool stop_flag_ = false;
  bool first_pressure = false;
  bool is_first_output = true;
  uint64_t last_output_time = 0;
  float depth_ = 0.0;
  PressureData last_pressure;
  PressureData curr_pressure;

  std::mutex lock_stop_;
  std::mutex lock_;
  std::mutex lock_pose_;

  std::mutex task_mut_;
  std::mutex imu_mut_;
  std::mutex soc_imu_mut_;
  std::mutex mag_mut_;
  std::mutex pressure_mut_;
  std::mutex ultra_mut_;
  std::mutex external_mut_;
  std::mutex left_front_ultra_mut_;
  std::mutex left_back_ultra_mut_;
  std::mutex relocation_mut_;

  //buildmap
  HJ_slam::HJ_mapping::BuildMapWithTwoSonarPtr build_map_ptr_;

  // relocalization
  aiper_relocalization_ns::Relocalization::Ptr relocalization_ptr_ = nullptr;
  Eigen::Matrix4d relocalization_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d water_entry_pose_ = Eigen::Vector3d::Zero();
  int is_relocalization_success_ = -1; // -1: not started, 0: failed, 1: success
  int is_build_frames_success_ = -1; // -1: not started, 0: failed, 1: success
  uint8_t relocalization_task_ = 0;
  bool is_relocalization_init_ = false;
  bool is_build_frames_done_ = false;

  // publishers
  hj_bf::HJPublisher sensor_record_pub_;
};
