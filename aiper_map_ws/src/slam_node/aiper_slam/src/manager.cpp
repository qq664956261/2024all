#include "tic_toc.h"
#include "manager.h"
#include "imu_data.h"
#include "soc_imu_data.h"
#include "encoder_data.h"
#include "pressure_data.h"
#include "mag_data.h"
#include "ultra_data.h"
#include "left_front_ultra_data.h"
#include "left_back_ultra_data.h"
#include "left_tof_data.h"
#include "navi_slam_data.h"
#include "optimized_para.h"
#include "common_tool.h"
#include "logic_dev/shm_data.h"
#include <chrono>   //计算时间

using namespace std::chrono;
#define debug_time

//#define Aiper_x9
#ifdef debug_time
static std::ofstream InfoImuIncc;
static std::ofstream InfoTofIncc;
static std::ofstream InfoLeftFrontIncc;
static std::ofstream InfoLeftBackIncc;
static std::ofstream InfoEncoderIncc;
#endif

SLAMManager::SLAMManager(const std::string &config_file_path, const std::string &output_directory, 
                         const std::string &relocalization_directory, const std::string &vocabulary)
    :stage_(STAGE_TRACKING_RELOCALIZATION) {
      std::cerr << "config_file_path" <<config_file_path <<std::endl;
       std::cerr << "relocalization_directory" <<relocalization_directory <<std::endl;
  std::string tracking_config_file_path;
#ifdef X9
  tracking_config_file_path = config_file_path + "/config/" + "config_x9.json";
  HJ_INFO("loading config: %s", tracking_config_file_path.c_str());
#endif
#ifdef T1_pro
  tracking_config_file_path = config_file_path + "/config/" + "config_t1_pro.json";
  HJ_INFO("Manager > load config: %s", tracking_config_file_path.c_str());
#endif

  std::ifstream file(tracking_config_file_path);
  if(!file.is_open()){
    HJ_ERROR("open json config failed...!!!, %s\n", tracking_config_file_path.c_str());
  }
  std::string jsonStr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  rapidjson::Document json_conf;
  json_conf.Parse(jsonStr.c_str());
  if (json_conf.HasParseError()) {
    HJ_ERROR("parse json config failed: %d", json_conf.GetParseError());
  }

  tracking_config_parameters_.loadParameters(json_conf["tracking"]);
  eskf_ptr_ = std::make_shared<HJ_slam::HJ_tracking::ErrorStateKalmanFilter>(tracking_config_parameters_);

  thread_ = std::thread(&SLAMManager::ThreadFunction, this);

  build_map_ptr_ = std::make_shared<HJ_slam::HJ_mapping::BuildMapWithTwoSonar>(json_conf);

  relocalization_ptr_ = std::make_shared<aiper_relocalization_ns::Relocalization>(relocalization_directory);

#ifdef debug_time
  InfoImuIncc.open("/userdata/hj/log/debug_imu.log");
  InfoTofIncc.open("/userdata/hj/log/debug_dtof.log");
  InfoLeftFrontIncc.open("/userdata/hj/log/debug_leftFront.log");
  InfoLeftBackIncc.open("/userdata/hj/log/debug_leftBack.log");
  InfoEncoderIncc.open("/userdata/hj/log/debug_encoder.log");
#endif

  sensor_record_pub_ = hj_bf::HJAdvertise<hj_interface::SensorDataRecord>("/sensor_data_record", 10);

}

void SLAMManager::ThreadFunction() {
  int ret = pthread_setname_np(pthread_self(), "aiper_tracking");
  if (ret == 0) {
    HJ_INFO("success name aiper_tracking %s", __FUNCTION__);
  } else {
    HJ_INFO("failed name aiper_tracking %s", __FUNCTION__);
  }

  std::unique_lock<std::mutex> lock_stop(lock_stop_);
  std::unique_lock<std::mutex> lock(lock_);
  while (!stop_flag_) {
    lock_stop.unlock();
    //step 1 get encoder measurement
    EncoderDataPtr encoder_output;
    uint64_t ts = 0;
    bool wait_res = false;
    wait_res =
            encoder_condition_.wait_for(lock, std::chrono::milliseconds(20), [&]() { return GetEncoderData(encoder_output, ts); });
    if (!wait_res) {
      lock_stop.lock();
      continue;
    }

    // check valid
    if (encoder_output->getTime() < static_cast<uint64_t>(0)) {
      lock_stop.lock();
      hj_interface::HealthCheckCode srv_msg;
      srv_msg.request.code_val = ENCODER_CHECK_ERROR;
      srv_msg.request.status = hj_interface::HealthCheckCodeRequest::ERROR;
      hj_bf::HjPushSrv(srv_msg);
      HJ_ERROR("encoder time is smaller than zero!");
      continue;
    }

    // step 2 get imu measurements
    bool get_imu = false;
    ImuMeasurements imu_measurements;
    imu_measurements.clear();
    get_imu = getMeasurementsContainingEdges(ts * 1.0e-6, imu_measurements, true);
    if (!get_imu) {
      HJ_INFO("ignore timestamp is %lf", encoder_output->getTime() / 1e6);
      lock_stop.lock();
      continue;
    }

    // step 3 get soc imu measurements
    bool get_soc_imu = false;
    SocImuMeasurements soc_imu_measurements;
    soc_imu_measurements.clear();
    get_soc_imu = getSocMeasurementsContainingEdges(ts * 1.0e-6, soc_imu_measurements, true);
    if (!get_soc_imu) {
//      lock_stop.lock();
//      continue;
    }

    // step 4 get mag measurement

    MagData mag_measurement(0, 0.f, 0.f, 0.f, 0.f);
    bool get_mag = getNearestMag(ts * 1.0e-6, mag_measurement);

    // step 5 get triple_ultra measurement
    UltraData ultra_measurement(0, 0, 0, 0, 0);
    bool get_ultra = getNearestUltra(ts * 1.0e-6, ultra_measurement);

    // step 6 get pressure measurement
    PressureData pressure_measurement(0, 0, 0);
    bool get_pressure = getNearestPressure(ts * 1.0e-6, pressure_measurement);
    checkPressureData(get_pressure, pressure_measurement);

    // step 7 get task id
    uint8_t curr_task_id = 0;
    bool get_task = getTaskID(curr_task_id);
    if (get_task) {
      task_id_ = get_task ? curr_task_id : -1;//1建图2重定位
      HJ_ERROR("current task id is %d", task_id_);
      GetTaskInfo();
    }

//    if (encoder_output->getTime() == 1724146812279997) {
//      HJ_INFO("current task id is water ground： building");
//      rotation_has_already_starting_ = true;
//      rotation_has_already_ending_ = true;
//      is_mag_init = true;
//      mag_init_success = true;
//      setStage(STAGE_TRACKING_MAPPING);
//    }

    // step 8 get navi to slam information
    NaviSlamData curr_navi_slam_info(0, 0, 0);
    bool get_navi = getNaviToSlamInfo(curr_navi_slam_info);
    if (get_navi) {
      HJ_INFO("Init process, slam has get navi information, the time is %ld, the mode is %d, "
               "the rotation state is %d", curr_navi_slam_info.time_, curr_navi_slam_info.mode_, curr_navi_slam_info.rotation_state_);
      if (curr_navi_slam_info.rotation_state_ == 1) {
        rotation_has_already_starting_ = true;
      }
      if (curr_navi_slam_info.rotation_state_ == 2) {
        rotation_has_already_ending_ = true;
      }
    }

//    HJ_INFO("curr_encoder_time is %lf", encoder_output->getTime() / 1e6);
//    HJ_INFO("find nearest mag %d", get_mag);
//    HJ_INFO("curr_task_id is %d", task_id_);

    // step 8 get relocalization pose
    // Eigen::Matrix4d relocalization_pose = Eigen::Matrix4d::Identity();
    // bool get_relocalization_data = getRelocalizationData(relocalization_pose);
    // getRelocalizationData(relocalization_pose);

    TicToc trackingOnceTime;
    updateState(encoder_output, imu_measurements, soc_imu_measurements, mag_measurement, get_mag, ultra_measurement, get_ultra, depth_);
    trackingOnceTime.toc();

    // printf("#updateState: %f, %ld\n", trackingOnceTime.toc(), encoder_output.time);
    lock_stop.lock();
  }
}

SLAMManager::~SLAMManager() {
  stop_flag_ = true;
  if (thread_.joinable()) {
    thread_.join();
    HJ_INFO("thread_ join");
  }
}

void SLAMManager::PutEncoderData(const ENcoderData &data_, const uint64_t timestamp_nanoseconds) {
#ifdef debug_time
  auto start = std::chrono::high_resolution_clock::now();
  auto start_time_t = std::chrono::system_clock::to_time_t(
          std::chrono::time_point_cast<std::chrono::seconds>(start)
  );
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;
  std::tm start_tm = *std::localtime(&start_time_t);
  std::stringstream ss;
  ss << std::put_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
  std::string start_time_str = ss.str();
  InfoEncoderIncc<< start_time_str.c_str() << " "<< data_.exposure_ts << std::endl;
#endif

  std::unique_lock<std::mutex> lock(encoder_lock_);
  encoder_queue.push(data_);
  encoder_condition_.notify_all();
}

void SLAMManager::SoftReset() {
  {
    std::unique_lock<std::mutex> lock(encoder_lock_);
    while (!encoder_queue.empty()) {
      encoder_queue.pop();
    }
  }

  {
    std::unique_lock<std::mutex> lock(imu_mut_);
    while (!measurements_.empty()) {
      measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(soc_imu_mut_);
    while (!soc_measurements_.empty()) {
      soc_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(mag_mut_);
    while (!mag_measurements_.empty()) {
      mag_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(pressure_mut_);
    while (!pressure_measurements_.empty()) {
      pressure_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(ultra_mut_);
    while (!ultra_measurements_.empty()) {
      ultra_measurements_.pop_front();
    }
  }
  //todo another class
  first_pressure = false;
  depth_ = 0.0;
  is_init_success_ = false;
  rotation_has_already_starting_ = false;
  rotation_has_already_ending_ = false;
  is_mag_init = false;
  mag_init_success = false;
  build_map_ptr_->reset();
  eskf_ptr_->softReset();
  resetRelocalization();
}

void SLAMManager::SoftSensorReset() {
  {
    std::unique_lock<std::mutex> lock(encoder_lock_);
    while (!encoder_queue.empty()) {
      encoder_queue.pop();
    }
  }

  {
    std::unique_lock<std::mutex> lock(imu_mut_);
    while (!measurements_.empty()) {
      measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(soc_imu_mut_);
    while (!soc_measurements_.empty()) {
      soc_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(mag_mut_);
    while (!mag_measurements_.empty()) {
      mag_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(pressure_mut_);
    while (!pressure_measurements_.empty()) {
      pressure_measurements_.pop_front();
    }
  }

  {
    std::unique_lock<std::mutex> lock(ultra_mut_);
    while (!ultra_measurements_.empty()) {
      ultra_measurements_.pop_front();
    }
  }
  //todo another class
  first_pressure = false;
  depth_ = 0.0;
  // build_map_ptr_->reset();
  // is_init_success_ = false;
}

void SLAMManager::PutHmdRelocalizationData(const Eigen::Matrix4d &relocalization_pose, const Eigen::Matrix4d &water_entry_pose, 
                                           uint8_t relocalization_result, uint8_t build_frames_result, const uint8_t &curr_task) {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  relocalization_pose_ = relocalization_pose;
  relocalization_task_ = curr_task;
  water_entry_pose_ = Eigen::Vector3d(water_entry_pose(0, 3), water_entry_pose(1, 3), water_entry_pose(2, 3));
  if (relocalization_result == 1) {
    is_relocalization_success_ = 1;
  } else if (relocalization_result == 2) {
    is_relocalization_success_ = 0;
  }

  if (build_frames_result == 1) {
    is_build_frames_success_ = 1;
  } else if (build_frames_result == 2) {
    is_build_frames_success_ = 0; 
  }
}

void SLAMManager::PutHmdTaskData(const uint8_t &curr_task) {
  addTask(curr_task);
}

void SLAMManager::PutHmdImuData(const IMUData &imu_data) {

  double scale = M_PI / 180.0;

  Eigen::Vector3d acc = Eigen::Vector3d(imu_data.acc_x / 1000.0 * 9.80, imu_data.acc_y / 1000.0 * 9.80, imu_data.acc_z / 1000.0 * 9.80);
  Eigen::Vector3d gyro = Eigen::Vector3d(imu_data.gyro_x / 100.0 * scale, imu_data.gyro_y / 100.0* scale, imu_data.gyro_z / 100.0* scale);
  Eigen::Vector3d euler = Eigen::Vector3d(imu_data.roll / 100.0, imu_data.pitch / 100.0, imu_data.yaw / 100.0);

  // printf("#imu_result: %f, %ld\n", euler[2], imu_data.timestamp);
  const ImuData m(imu_data.timestamp, gyro, acc, euler);

#ifdef debug_time
  auto start = std::chrono::high_resolution_clock::now();
  auto start_time_t = std::chrono::system_clock::to_time_t(
          std::chrono::time_point_cast<std::chrono::seconds>(start)
  );
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;
  std::tm start_tm = *std::localtime(&start_time_t);
  std::stringstream ss;
  ss << std::put_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
  std::string start_time_str = ss.str();
  InfoImuIncc<< start_time_str.c_str() << " "<< imu_data.timestamp << std::endl;
#endif

  addImuMeasurement(m);

}

void SLAMManager::PutHmdSocImuData(const SOCIMUData &soc_imu_data) {

  Eigen::Vector3d acc = Eigen::Vector3d(soc_imu_data.acc_x, soc_imu_data.acc_y, soc_imu_data.acc_z);
  Eigen::Vector3d gyro = Eigen::Vector3d(soc_imu_data.gyro_x, soc_imu_data.gyro_y, soc_imu_data.gyro_z);

  const SocImuData m(soc_imu_data.timestamp, gyro, acc);
  addSocImuMeasurement(m);
  if (soc_imu_data.timestamp > 1679943865708383) {
    ending = true;
  }
}

void SLAMManager::PutHmdMagData(const MAGData &mag_data) {
  // segual pro mag
  float mag_x = static_cast<float>(mag_data.mag_x);
  float mag_y = static_cast<float>(mag_data.mag_y);
  float mag_z = static_cast<float>(mag_data.mag_z);

  // printf("#mag_result: %f, %f, %f, %ld\n", mag_x_, mag_y_, mag_z_, mag_data.timestamp);
  const MagData m(mag_data.timestamp, mag_x, mag_y, mag_z, 0.f);
  addMagMeasurement(m);
}

void SLAMManager::PutHmdUltraData(const ULtraData &ultra_data) {
  uint32_t front_l= ultra_data.front_distance_l;
  uint32_t front_m = ultra_data.front_distance_m;
  uint32_t front_r = ultra_data.front_distance_r;
  uint8_t status = ultra_data.status;

  // printf("#ultra_result: %d, %d, %d, %ld\n", front_distance_, mid_distance_ , back_distance_, ultra_data.timestamp);
  const UltraData u(ultra_data.timestamp, front_l, front_m, front_r, status);
  addUltraMeasurement(u);
}

void SLAMManager::PutHmdDepthData(const DEPData &dep_data) {
  int32_t pressure = static_cast<int32_t>(dep_data.pressure);
  int32_t temperature = static_cast<int32_t>(dep_data.temperature);

  // printf("#pressure_result: %lu, %lu, %ld\n", pressure_, temperature_, mag_data.timestamp);
  const PressureData p(dep_data.timestamp, pressure, temperature);
  addPressureMeasurement(p);
}

#ifdef X9
void SLAMManager::PutHmdLeftFrontUltraData(const LEFTFrontData &left_front_data) {
#ifdef debug_time
  auto start = std::chrono::high_resolution_clock::now();
  auto start_time_t = std::chrono::system_clock::to_time_t(
          std::chrono::time_point_cast<std::chrono::seconds>(start)
  );
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;
  std::tm start_tm = *std::localtime(&start_time_t);
  std::stringstream ss;
  ss << std::put_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
  std::string start_time_str = ss.str();
  InfoLeftFrontIncc<< start_time_str.c_str() << " "<< left_front_data.timestamp << std::endl;
#endif
  build_map_ptr_->setLeftFrontSonarData(left_front_data);
}
void SLAMManager::PutHmdLeftBackUltraData(const LEFTBackData &left_back_data) {
#ifdef debug_time
  auto start = std::chrono::high_resolution_clock::now();
  auto start_time_t = std::chrono::system_clock::to_time_t(
          std::chrono::time_point_cast<std::chrono::seconds>(start)
  );
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;
  std::tm start_tm = *std::localtime(&start_time_t);
  std::stringstream ss;
  ss << std::put_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
  std::string start_time_str = ss.str();
  InfoLeftBackIncc<< start_time_str.c_str() << " "<< left_back_data.timestamp << std::endl;
#endif
  build_map_ptr_->setLeftBackSonarData(left_back_data);
}
#endif

#ifdef T1_pro
void SLAMManager::PutHmdLeftTofData(const LEFTTofData &left_tof_data) {
  #ifdef debug_time
  auto start = std::chrono::high_resolution_clock::now();
  auto start_time_t = std::chrono::system_clock::to_time_t(
          std::chrono::time_point_cast<std::chrono::seconds>(start)
  );
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()) % 1000;
  std::tm start_tm = *std::localtime(&start_time_t);
  std::stringstream ss;
  ss << std::put_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
  std::string start_time_str = ss.str();
  InfoTofIncc<< start_time_str.c_str() << " "<< left_tof_data.timestamp << std::endl;
#endif
  build_map_ptr_->setLeftTofData(left_tof_data);
}
#endif

void SLAMManager::PutNaviSlamData(const NAVISlamData &navi_slam_data) {
  const NaviSlamData data(navi_slam_data.timestamp, navi_slam_data.mode, navi_slam_data.rotation_state);
  addNaviSlamMeasurement(data);
}

bool SLAMManager::addTask(const uint8_t &task) {
  std::unique_lock<std::mutex> lock(task_mut_);
  task_lists_.push(task);  // new measurement is at the front of the list!
  return true;
}

void SLAMManager::GetTaskInfo() {
  //水面任务
  if (task_id_ == 11) {
    HJ_INFO("current task id is pool surface");
    SoftReset();
    exportRecordActionToMiddleware(task_id_);
    stage_ = STAGE_TRACKING;
  }
  //爬墙任务
  if (task_id_ == 13) {
    HJ_INFO("current task id is pool wall");
    SoftReset();
    exportRecordActionToMiddleware(task_id_);
    stage_ = STAGE_TRACKING;
  }
  //水线任务
  if (task_id_ == 14) {
    HJ_INFO("current task id is pool line");
    SoftReset();
    exportRecordActionToMiddleware(task_id_);
    stage_ = STAGE_TRACKING;
  }
  //池底任务
  if (task_id_ == 15) {
    HJ_INFO("current task id is water ground");
    SoftReset();
    exportRecordActionToMiddleware(task_id_);
    stage_ = STAGE_TRACKING;
  }

  if (task_id_ == 1) {
    HJ_INFO("current task id is water ground: building");
    setStage(STAGE_TRACKING_MAPPING);
  }
  if (task_id_ == 2) {
    setStage(STAGE_TRACKING_RELOCALIZATION);
    relocalization_ptr_->start(task_id_);
    HJ_INFO("current task id is water ground: relocation");
  }
  if (task_id_ == 26) {
    HJ_INFO("current task id is water ground: recall");
    SoftReset();
    stage_ = STAGE_TRACKING;
  }
  if (task_id_ == 28) {
    setStage(STAGE_TRACKING_RELOCALIZATION);
    relocalization_ptr_->start(task_id_);
    HJ_INFO("current task id is recall with map: relocation");
  }
  if (task_id_ == 35) {
    exportRecordActionToMiddleware(task_id_);
    HJ_INFO("current task id is 35: task end");
  }
}

bool SLAMManager::addImuMeasurement(const ImuData &m) {
  std::unique_lock<std::mutex> lock(imu_mut_);
  measurements_.push_front(m);  // new measurement is at the front of the list!
  if (measurements_.size() > 1000) {
    measurements_.pop_back();
  }
  return true;
}

bool SLAMManager::addSocImuMeasurement(const SocImuData &m) {
  std::unique_lock<std::mutex> lock(soc_imu_mut_);
  soc_measurements_.push_front(m);  // new measurement is at the front of the list!
  if (soc_measurements_.size() > 1000) {
    soc_measurements_.pop_back();
  }
  return true;
}

bool SLAMManager::addMagMeasurement(const MagData &m) {
  std::unique_lock<std::mutex> lock(mag_mut_);
  mag_measurements_.push_front(m);  // new measurement is at the front of the list!
  if(mag_measurements_.size() > 100) { //todo
    mag_measurements_.pop_back();
  }
  return true;
}

bool SLAMManager::addPressureMeasurement(const PressureData &p) {
  std::unique_lock<std::mutex> lock(pressure_mut_);
  pressure_measurements_.push_front(p);  // new measurement is at the front of the list!
  if(pressure_measurements_.size() > 100) { //todo
    pressure_measurements_.pop_back();
  }
  return true;
}

bool SLAMManager::addUltraMeasurement(const UltraData &u) {
  std::unique_lock<std::mutex> lock(ultra_mut_);
  ultra_measurements_.push_front(u);  // new measurement is at the front of the list!
  if(ultra_measurements_.size() > 100) { //todo
    ultra_measurements_.pop_back();
  }
  return true;
}

bool SLAMManager::addNaviSlamMeasurement(const NaviSlamData &data) {
  std::unique_lock<std::mutex> lock(external_mut_);
  navi_slam_measurements_.push(data);  // new measurement is at the front of the list!
  if(navi_slam_measurements_.size() > 10) { //todo
    navi_slam_measurements_.pop();
  }
  return true;
}

//PoseDataPtr SLAMManager::getPose() {
//  return output_pose_;
//}

void SLAMManager::sendFusionPose(PoseDataPtr &output_pose) {
  if (output_pose == nullptr) {
    return;
  }
  std::unique_lock<std::mutex> lock(output_lock_);
  output_pose_queue.push(output_pose);
  if (output_pose_queue.size() > 100) {
    output_pose_queue.pop();
  }
}

void SLAMManager::sendMagAngle(MagOutputDataPtr &output_mag_angle) {
  if (output_mag_angle == nullptr) {
    return;
  }
  std::unique_lock<std::mutex> lock(output_mag_lock_);
  output_mag_angle_queue.push(output_mag_angle);
  if (output_mag_angle_queue.size() > 100) {
    output_mag_angle_queue.pop();
  }
}

SLAMManager::Stage SLAMManager::getStage() {
  return stage_;
}

std::list<MapPointPtr> SLAMManager::getICPTargetCloud() {
  return build_map_ptr_->getICPTargetCloud();
}


bool SLAMManager::GetEncoderData(EncoderDataPtr &encoder_data, uint64_t &ts) {
  std::unique_lock<std::mutex> lock(encoder_lock_);
  if (encoder_queue.empty()) {
//    HJ_WARN("don't have any encoder data!");
    return false;
  }
  ENcoderData first_encoder_data = encoder_queue.front();
  encoder_queue.pop();
  if (!encoder_queue.empty()) {
    HJ_WARN("encoder leave size %ld\n", encoder_queue.size());
  }
  lock.unlock();
  uint64_t time = first_encoder_data.exposure_ts;
  double v_left = first_encoder_data.v_l;
  double v_right = first_encoder_data.v_r;

  encoder_data = std::make_shared<EncoderData>(time, v_left, v_right);
  ts = time;
#ifdef X9
  usleep(2000);
#endif
#ifdef T1_pro
  usleep(4000);
#endif
  return true;
}

bool SLAMManager::getMeasurementsContainingEdges(const double frame_timestamp,  // seconds
                                                 ImuMeasurements& extracted_measurements,
                                                 const bool remove_measurements) {
  std::unique_lock<std::mutex> lock(imu_mut_);
  if (measurements_.empty()) {
    HJ_WARN("don't have any imu measurements!");
    return false;
  }

  // Subtract camera delay to get imu timestamp.
  double wait_time = 0.0;
#ifdef X9
  wait_time = 2.0 * 1e-3;
#endif
#ifdef T1_pro
  wait_time = 6.0 * 1e-3;
#endif
  const double t = frame_timestamp - wait_time;

  // Find the first measurement newer than frame_timestamp,
  // note that the newest measurement is at the front of the list!
  ImuMeasurements::iterator it = measurements_.begin();

  for (; it != measurements_.end(); ++it) {

    if ((it->time * 1e-6) < t) {
//      uint64_t t_encoder = static_cast<uint64_t >(t * 1e6);
//      uint64_t t_imu = static_cast<uint64_t>(it->time);
      // std::cout<<"note!, encoder timestamp = "<<t_encoder<<std::endl;
      // std::cout<<"note!, imu timestamp = "<<t_imu<<std::endl;
      if (it == measurements_.begin()) {

        return false;
      }
      // decrement iterator again to point to element >= t
      --it;
      break;
    }
  }

  // copy affected measurements
  extracted_measurements.insert(extracted_measurements.begin(), it, measurements_.end());

  // check
  if (extracted_measurements.size() < 2) {
    HJ_WARN("need older imu measurements!");
    extracted_measurements.clear();
    return false;
  }

  for (auto it_imu : extracted_measurements) {
    if (fabs(it_imu.angular_velocity_[0]) < 1e-3 &&  fabs(it_imu.angular_velocity_[1]) < 1e-3 && fabs(it_imu.angular_velocity_[2]) < 1e-3 &&
        fabs(it_imu.linear_acceleration_[0]) < 1e-3 &&  fabs(it_imu.linear_acceleration_[1]) < 1e-3 && fabs(it_imu.linear_acceleration_[2]) < 1e-3) {
      HJ_INFO("imu is all zero, please keep machine static");
      return false;
    }
  }

  if (static_cast<int>(measurements_.end() -it) > 2) {
    // delete measurements that will not be used anymore (such that we keep it+1,
    // the first frame with smaller timestamp (earlier) than frame_timestamp,
    // which will be used in interpolation in next iteration
    if (it + 2 <= measurements_.end()) {
      measurements_.erase(it + 2, measurements_.end());
    }
  }
  return true;
}

bool SLAMManager::getSocMeasurementsContainingEdges(const double frame_timestamp,  // seconds
                                                 SocImuMeasurements& extracted_measurements,
                                                 const bool remove_measurements) {
  std::unique_lock<std::mutex> lock(soc_imu_mut_);
  if (soc_measurements_.empty()) {
//    HJ_WARN("don't have any soc imu measurements!");
    return false;
  }

  const double t = frame_timestamp;

  SocImuMeasurements::iterator it = soc_measurements_.begin();

  for (; it != soc_measurements_.end(); ++it) {

    if ((it->time * 1e-6) < t) {
      if (it == soc_measurements_.begin()) {

        return false;
      }
      // decrement iterator again to point to element >= t
      --it;
      break;
    }
  }

  // copy affected measurements
  extracted_measurements.insert(extracted_measurements.begin(), it, soc_measurements_.end());

  // check
  if (extracted_measurements.size() < 2) {
    HJ_WARN("need older soc imu measurements!");
    extracted_measurements.clear();
    return false;
  }

  if (static_cast<int>(soc_measurements_.end() -it) > 2) {
    if (it + 2 <= soc_measurements_.end()) {
      soc_measurements_.erase(it + 2, soc_measurements_.end());
    }
  }
  return true;
}


bool SLAMManager::getNearestMag(const double frame_timestamp, MagData &mag_measurement) {
  std::unique_lock<std::mutex> lock(mag_mut_);
  if (mag_measurements_.empty()) {
//    HJ_WARN("don't have any mag measurements!");
    return false;
  }

  // Subtract camera delay to get imu timestamp.
  const uint64_t t_64 = static_cast<uint64_t>(frame_timestamp * 1e6);

  // Find the first measurement newer than frame_timestamp,
  // note that the newest measurement is at the front of the list!
  int find_nearest_count = 0;
  double min_ = DBL_MAX;
  MagData mag_candidate(0, 0.f, 0.f, 0.f, 0.f);
  MagMeasurements::iterator it_begin = mag_measurements_.begin();
  while (find_nearest_count < 5) {
    double diff_time = fabs(static_cast<double>(t_64) - static_cast<double>(it_begin->time));
    if (diff_time < min_) {
      min_ = diff_time;
      mag_candidate = (*it_begin);
    }
    it_begin++;
    find_nearest_count++;
  }

  double diff_min_time = fabs(static_cast<double>(t_64) - static_cast<double>(mag_candidate.time));
  if (diff_min_time < 10000.0) {
    mag_measurement = mag_candidate;
    return true;
  }
  return false;

}


bool SLAMManager::getNearestUltra(const double frame_timestamp, UltraData &ultra_measurement) {
  std::unique_lock<std::mutex> lock(ultra_mut_);
  if (ultra_measurements_.empty()) {
//    HJ_WARN("don't have any ultra measurements!");
    return false;
  }

  // Subtract camera delay to get imu timestamp.
  const uint64_t t_64 = static_cast<uint64_t>(frame_timestamp * 1e6);

  // Find the first measurement newer than frame_timestamp,
  // note that the newest measurement is at the front of the list!
  int find_nearest_count = 0;
  double min_ = DBL_MAX;
  UltraData ultra_candidate(0, 0, 0, 0, 0);
  UltraDataMeasurements::iterator it_begin = ultra_measurements_.begin();
  while (find_nearest_count < 5) {
    double diff_time = fabs(static_cast<double>(t_64) - static_cast<double>(it_begin->time_));
    if (diff_time < min_) {
      min_ = diff_time;
      ultra_candidate = (*it_begin);
    }
    it_begin++;
    find_nearest_count++;
  }

  double diff_min_time = fabs(static_cast<double>(t_64) - static_cast<double>(ultra_candidate.time_));
  if (diff_min_time < 10000.0) {
    ultra_measurement = ultra_candidate;
    return true;
  }
  return false;

}

bool SLAMManager::getNearestPressure(const double frame_timestamp, PressureData &press_measurement) {
  std::unique_lock<std::mutex> lock(pressure_mut_);
  if (pressure_measurements_.empty()) {
//    HJ_WARN("don't have any pressure measurements!");
    return false;
  }

  // Subtract camera delay to get imu timestamp.
  const uint64_t t_64 = static_cast<uint64_t>(frame_timestamp * 1e6);

  // Find the first measurement newer than frame_timestamp,
  // note that the newest measurement is at the front of the list!
  int find_nearest_count = 0;
  double min_ = DBL_MAX;
  PressureData pressure_candidate(0, 0, 0);
  PressureMeasurements::iterator it_begin = pressure_measurements_.begin();
  while (find_nearest_count < 5) {
    double diff_time = fabs(static_cast<double>(t_64) - static_cast<double>(it_begin->time));
    if (diff_time < min_) {
      min_ = diff_time;
      pressure_candidate = (*it_begin);
    }
    it_begin++;
    find_nearest_count++;
  }

  double diff_min_time = fabs(static_cast<double>(t_64) - static_cast<double>(pressure_candidate.time));
  // std::cout<<"diff time of encoder and pressure is equal to "<<diff_min_time<<std::endl;
  if (diff_min_time < 20000.0) {
    press_measurement = pressure_candidate;
    return true;
  }
  return false;

}

void SLAMManager::checkPressureData(bool &get_pressure, PressureData &pressure_measurement) {
  if (get_pressure) {
    if (!first_pressure) {
      last_pressure = pressure_measurement;
      first_pressure = true;
    }
    else {
      curr_pressure = pressure_measurement;
      float delta_z = curr_pressure.calDeltaZ(last_pressure, curr_pressure);
      depth_ += delta_z;
      last_pressure = curr_pressure;
    }
  }

}

bool SLAMManager::getTaskID(uint8_t &task_id) {

  std::unique_lock<std::mutex> lock(task_mut_);
  if (task_lists_.empty()) {
//    HJ_WARN("don't have any task id!");
    return false;
  }

  task_id = task_lists_.front();
  task_lists_.pop();
  return true;
}

bool SLAMManager::getNaviToSlamInfo(NaviSlamData &data_info) {

  std::unique_lock<std::mutex> lock(external_mut_);
  if (navi_slam_measurements_.empty()) {
//    HJ_WARN("don't have any task id!");
    return false;
  }

  data_info = navi_slam_measurements_.front();
  navi_slam_measurements_.pop();
  return true;
}

bool SLAMManager::isInitSuccess() {
  return is_init_success_;
}

void SLAMManager::exportRelocResultToESKF() {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  eskf_ptr_->setRelocalizationSuccess();

  Eigen::Vector3d relocalization_position = relocalization_pose_.block<3,1>(0,3);

  Eigen::Vector3d euler_after = Quaternion2EulerAngles(Eigen::Quaterniond(relocalization_pose_.block<3,3>(0,0)));
  HJ_INFO("euler_after = %f, %f, %f", euler_after[2] * kRadian2Degree, euler_after[1] * kRadian2Degree, euler_after[0] * kRadian2Degree);
  Eigen::Matrix4d curr_pose = eskf_ptr_->getPose();
  Eigen::Vector3d euler_before = Quaternion2EulerAngles(Eigen::Quaterniond(curr_pose.block<3,3>(0,0)));
  HJ_INFO("euler_before = %f, %f, %f", euler_before[2] * kRadian2Degree, euler_before[1] * kRadian2Degree, euler_before[0] * kRadian2Degree);

  Eigen::Vector3d euler_relocalization_corrected = Eigen::Vector3d(euler_after[2] - euler_before[2], euler_after[1] - euler_before[1], euler_after[0] - euler_before[0]);

  if (euler_relocalization_corrected[2] > M_PI) {
    euler_relocalization_corrected[2] -= 2 * M_PI;
  }
  if (euler_relocalization_corrected[2] < -M_PI) {
    euler_relocalization_corrected[2] += 2 * M_PI;
  }
  HJ_INFO("euler_corrected = %f, %f, %f", euler_relocalization_corrected[0] * kRadian2Degree, euler_relocalization_corrected[1] * kRadian2Degree, euler_relocalization_corrected[2] * kRadian2Degree);

  eskf_ptr_->setRelocalizationPose(relocalization_position, euler_relocalization_corrected);
}

void SLAMManager::exportRelocResultToApp(bool success) {
  hj_bf::HJClient slam_clinet = hj_bf::HJCreateClient<hj_interface::SlamNaviWorkResult>("/slam_navi_action_result_service");
  hj_interface::SlamNaviWorkResult curr_task;
  curr_task.request.action_cmd = 2;
  curr_task.request.action_result = success ? 11 : 12;
  if(slam_clinet.call(curr_task)) {
    HJ_INFO("client relocalization result is successfully published");
  }
  else{
    HJ_INFO("client relocalization result is failed published");
  }
}

void SLAMManager::exportRelocResultToNavi(bool success) {
  HJ_INFO("Reloc > export relocalization result to navi... task_id: %d, success:%d, position: %f, %f, %f", 
          relocalization_task_, success, water_entry_pose_(0), water_entry_pose_(1), water_entry_pose_(2));
  hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::AlgWorkResult>("/alg_work_result_service");
  hj_interface::AlgWorkResult reloc_result;
  reloc_result.request.action_cmd = relocalization_task_;
  reloc_result.request.action_result = success ? 1 : 2;
  reloc_result.request.point.x = water_entry_pose_(0);
  reloc_result.request.point.y = water_entry_pose_(1);
  reloc_result.request.point.z = water_entry_pose_(2);
  client.call(reloc_result);
}

void SLAMManager::exportMapToAPP(const std::list<MapPointPtr> &map_point_list, const Eigen::Vector3f &water_entry_position) {
  int map_point_size = static_cast<int>(map_point_list.size());
  if(map_point_size > shm_data::MAX_POINT - 10) {
    HJ_ERROR("[reloc]: map size is too large, please increase MAX_POINT, map size: %zu", map_point_list.size());
    return;
  }
  shm_data::MapData map_data[shm_data::MAX_POINT] = {};
  map_data[0].x = static_cast<int32_t>(map_point_list.size());

  // update water entry position in map data
  map_data[0].y = 1;
  map_data[1].x = static_cast<int32_t>(water_entry_position[0] * 1000.0 + 0.5);
  map_data[1].y = static_cast<int32_t>(water_entry_position[1] * 1000.0 + 0.5);
  map_data[1].z = static_cast<int32_t>(water_entry_position[2] * 1000.0 + 0.5);

  int i = 10;
  for (auto it_cloud = map_point_list.begin(); it_cloud != map_point_list.end(); ++it_cloud) {
    map_data[i].x = static_cast<int32_t>((*it_cloud)->getCor_x() * 1000.0 + 0.5);
    map_data[i].y = static_cast<int32_t>((*it_cloud)->getCor_y() * 1000.0 + 0.5);
    map_data[i].z = static_cast<int32_t>((*it_cloud)->getCor_z() * 1000.0 + 0.5);
    i++;
  }
  hj_bf::setVariable("map_data_shm", map_data);
  HJ_INFO("set map varible to share memory, map size: %d, i: %d", static_cast<int>(map_data[0].x), i);
}

void SLAMManager::exportRecordActionToMiddleware(const uint8_t& task_id) {
  hj_interface::SensorDataRecord sensor_recoder_msg;
  sensor_recoder_msg.action_cmd = task_id;
  sensor_record_pub_.publish(sensor_recoder_msg);
}

void SLAMManager::getMapFromAPPTest() {
  shm_data::MapData map_data_temp[shm_data::MAX_POINT] = {};
  bool ret = hj_bf::getVariable("map_data_shm", map_data_temp);
  if (ret) {
    HJ_INFO("get map varible from share memory, map size: %d", map_data_temp[0].x);
  }

  else {
    HJ_ERROR("get map varible from share memory failed");
  }
}


void SLAMManager::waterEntryPointUpdate() {
  std::list<MapPointPtr> target_point_cloud = build_map_ptr_->getICPTargetCloud();

  Eigen::Vector3f water_entry_position = Eigen::Vector3f::Zero();
  water_entry_position[0] = water_entry_pose_(0);
  water_entry_position[1] = water_entry_pose_(1);
  water_entry_position[2] = water_entry_pose_(2);

  exportMapToAPP(target_point_cloud, water_entry_position);
  getMapFromAPPTest();
}

bool SLAMManager::needsRelocalization() {
  if (task_id_ == 2) {
    HJ_INFO("Reloc > needs relocalization!!!");
    task_id_ = -1;  // reset task id
    return true;
  }
  return false;
}

bool SLAMManager::isRelocalizationSuccess() {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  if (is_relocalization_success_ == 1) {
    HJ_INFO("Reloc > relocalization success, stop Relocalization!!!");
    is_relocalization_success_ = -1; // reset relocalization result
    return true;
  } else {
    return false;
  }
}

bool SLAMManager::isRelocalizationFailed() {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  if (is_relocalization_success_ == 0) {
    HJ_INFO("Reloc > relocalization failed, stop Relocalization!!!");
    is_relocalization_success_ = -1; // reset relocalization result
    return true;
  } else {
    return false;
  }
}

bool SLAMManager::isBuildFramesSuccess() {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  if (is_build_frames_success_ == 1) {
    HJ_INFO("Frames > build frames success, stop build frames!!!");
    is_build_frames_success_ = -1; // reset relocalization result
    return true;
  } else {
    return false;
  }
}

bool SLAMManager::isBuildFramesFailed() {
  std::unique_lock<std::mutex> lock(relocation_mut_);
  if (is_build_frames_success_ == 0) {
    HJ_INFO("Frames > build frames failed, stop build frames!!!");
    is_build_frames_success_ = -1; // reset relocalization result
    return true;
  } else {
    return false;
  }
}

void SLAMManager::resetRelocalization() {
  relocalization_ptr_->stop();
  relocalization_pose_ = Eigen::Matrix4d::Identity();
  water_entry_pose_ = Eigen::Vector3d::Identity();
  is_relocalization_success_ = -1; // -1: not started, 0: failed, 1: success
  is_build_frames_success_ = -1; // -1: not started, 0: failed, 1: success
  relocalization_task_ = 0;
  is_build_frames_done_ = false;
}

bool SLAMManager::needsMapping() {
  if (task_id_ == 1) {
    HJ_INFO("[Mapping] needs mapping!!!");
    task_id_ = -1;  // reset task id
    return true;
  }
  return false;
}

bool SLAMManager::isMappingFinished() {
  HJ_INFO("Mapping > mapping finished!!!");
  if (build_map_ptr_->isMappingFinished()) {
    return true;
  } else {
    return false;
  }
}

int SLAMManager::releaseRelocalization() {
  relocalization_ptr_.reset();
  return 1;
}

void SLAMManager::setStage(const Stage& stage) {
  stage_ = stage;
}

void SLAMManager::trackingOnceThreeDof(EncoderDataPtr &curr_encoder, ImuMeasurements &imus) {
  auto three_dof_pose = eskf_ptr_->process_three_dof(curr_encoder, imus);
  sendFusionPose(three_dof_pose);

}

void SLAMManager::collectMagData(EncoderDataPtr &curr_encoder, MagData &mag, bool &is_mag) {
  //磁力计坐标系转到imu坐标系
  Eigen::Matrix3d Tw_mag;
  Tw_mag << 0, -1, 0, -1, 0, 0, 0, 0, -1;
  Eigen::Matrix<double, 3, 1> mag_mat;
  mag_mat << mag.mag_x, mag.mag_y, mag.mag_z;
  Eigen::Matrix<double, 3, 1> mag_world = Tw_mag * mag_mat;
  Eigen::Vector3d mag_w = Eigen::Vector3d(mag_world[0], mag_world[1], mag_world[2]);
  eskf_ptr_->collectMagData(curr_encoder, mag_w, is_mag, rotation_has_already_ending_);
}

bool SLAMManager::computeMagOffset(Eigen::Vector3d &magOffset) {
  int mag_collected_size = static_cast<int>(eskf_ptr_->time_euler_mag.size());
  if (mag_collected_size < 10) {
    HJ_INFO("collect ending, the mag size is too less, the mag size is = %d", mag_collected_size);
    return false;
  }
  magOffset = eskf_ptr_->calMagOffset();
  return true;
}

void SLAMManager::setMagOffset(Eigen::Vector3d &magOffset) {
  eskf_ptr_->setMagOffset(magOffset);
}

PoseDataPtr SLAMManager::trackingOnce(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus,
                               MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra, 
                               float &depth_z) {
  PoseDataPtr output_pose = eskf_ptr_->process(curr_encoder, imus, soc_imus, mag, is_mag, ultra_measurement, get_ultra);
  if (is_mag && is_mag_init && mag_init_success) {
    MagOutputDataPtr output_mag_angle = eskf_ptr_->estimateMagYaw(curr_encoder, mag, is_mag);
    sendMagAngle(output_mag_angle);
  }

  eskf_ptr_->doDepthFusion(depth_z);
  output_pose->p_[2] = depth_z;

  sendFusionPose(output_pose);
  // build_map_ptr_->setFusionPose(output_pose);

  std::vector<Eigen::Matrix4d> icp_trans_list;
  bool get_icp_trans = build_map_ptr_->getIcpTransform(icp_trans_list);
  if (get_icp_trans) {
    Eigen::Matrix4d icp_transform = icp_trans_list[0];
    Eigen::Matrix4d curr_pose = eskf_ptr_->getPose();
    Eigen::Vector3d euler_before = Quaternion2EulerAngles(Eigen::Quaterniond(curr_pose.block<3,3>(0,0)));
    HJ_INFO("before change = %f, %f, %f", euler_before[2] * kRadian2Degree, euler_before[1] * kRadian2Degree, euler_before[0] * kRadian2Degree);
    Eigen::Matrix4d pose_temp = Eigen::Matrix4d::Identity();
    pose_temp = icp_transform * curr_pose;

    Eigen::Vector3d euler_after = Quaternion2EulerAngles(Eigen::Quaterniond(pose_temp.block<3,3>(0,0)));
    HJ_INFO("after change = %f, %f, %f", euler_after[2] * kRadian2Degree, euler_after[1] * kRadian2Degree, euler_after[0] * kRadian2Degree);
    Eigen::Vector3d euler_icp_corrected = Eigen::Vector3d(euler_after[2] - euler_before[2], euler_after[1] - euler_before[1], euler_after[0] - euler_before[0]);
    HJ_INFO("corrected = %f, %f, %f", euler_icp_corrected[0] * kRadian2Degree, euler_icp_corrected[1] * kRadian2Degree, euler_icp_corrected[2] * kRadian2Degree);
    eskf_ptr_->setIcpResult(euler_icp_corrected);
    eskf_ptr_->AlterPose(pose_temp);
    eskf_ptr_->time_angle_pose_front_ultra.clear();
  }
  return output_pose;
}

void SLAMManager::relocalizeOnce() {
  if (isRelocalizationSuccess()) {
    HJ_INFO("Reloc > relocalization success, stop Relocalization!!!");
    // init
    relocalization_ptr_->stop();
  
    exportRelocResultToESKF();
    if (relocalization_task_ == 2) {                // 入水重定位
      exportRelocResultToApp(true);
      waterEntryPointUpdate();
    } else if (relocalization_task_ == 28) {        // 召回重定位
      exportRelocResultToNavi(true);
    }
    
    setStage(STAGE_TRACKING);
  } else if (isRelocalizationFailed()) {
    HJ_INFO("Reloc > relocalization failed, stop Relocalization!!!");
    relocalization_ptr_->stop();
    
    if (relocalization_task_ == 2) { 
      exportRelocResultToApp(false);
    } else if (relocalization_task_ == 28) {      // 召回重定位
      exportRelocResultToNavi(false);
    }

    setStage(STAGE_TRACKING);
  }
}

void SLAMManager::mappingOnce(const PoseDataPtr& pose) {
  // TODO(edan): tmp add build frames
  if (is_relocalization_init_ == false && is_build_frames_done_ == false) {
    relocalization_ptr_->start(task_id_);
    is_relocalization_init_ = true;
  }

  if (isBuildFramesSuccess()) {
    relocalization_ptr_->stop();
    is_relocalization_init_ = false;
    is_build_frames_done_ = true;
  } else if (isBuildFramesFailed()) {
    relocalization_ptr_->stop();
    is_relocalization_init_ = false;
    is_build_frames_done_ = true;
  }
  // end tmp add build frames
  bool stop_sending_pose = build_map_ptr_->isStopSendingMsg();

  if (!stop_sending_pose) {
    build_map_ptr_->setFusionPose(pose);
  }
  else {
    if (build_map_ptr_->isMapBuildingSuccess()) {
      Eigen::Vector3d t_external = Eigen::Vector3d::Zero();
    #ifdef X9
      Eigen::Vector3d t_bm_x9 = Eigen::Vector3d(-0.11, 0.0, 0.0);
      t_external = t_bm_x9;
    #endif
    #ifdef T1_pro
      Eigen::Vector3d t_bm_t1_pro = Eigen::Vector3d(0.0, 0.0, 0.0);
      t_external = t_bm_t1_pro;
    #endif
      water_entry_pose_ = t_external;

    }
  }

}

void SLAMManager::updateState(EncoderDataPtr &curr_encoder, ImuMeasurements &imus, SocImuMeasurements &soc_imus,
                               MagData &mag, bool &is_mag, UltraData &ultra_measurement, bool &get_ultra, 
                               float &depth_z) {
  //if not rotation ready, output 3dof pose
  //  if (!rotation_has_already_starting_) {
  //    trackingOnceThreeDof(curr_encoder, imus);
  //    return;
  //  }
  //  if (rotation_has_already_starting_ && !rotation_has_already_ending_) {
  //    trackingOnceThreeDof(curr_encoder, imus);
  //    //这里已经转过磁力计坐标系，但还没经过roll pitch角度的矫正
  //    collectMagData(curr_encoder, mag, is_mag);
  //    return;
  //  }
  //  //now rotation is ready, we must initialize and output 6dof pose
  //  if (rotation_has_already_starting_ && rotation_has_already_ending_ && !is_mag_init) {
  //    Eigen::Vector3d mag_offset = Eigen::Vector3d::Zero();
  //    bool is_mag_calib_valid = computeMagOffset(mag_offset);
  //    if (is_mag_calib_valid) {
  //      setMagOffset(mag_offset);
  //      HJ_INFO("mag_offset = %lf, %lf, %lf", mag_offset[0], mag_offset[1], mag_offset[2]);
  //      is_mag_init = true;
  //      mag_init_success = true;
  //    }
  //    else {
  //      HJ_ERROR("mag calib is failure");
  //      is_mag_init = true;
  //      mag_init_success = false;
  //    }
  //  }

  // get the fusion pose after measurement
  //frequency is about 50hz, the same as encoder frequency
  // if (!is_init_success_ && is_mag_init) {
  if (!is_init_success_) {
    is_init_success_ = eskf_ptr_->initialize(curr_encoder, imus, mag, is_mag, mag_init_success);
    return;
  }

  switch (stage_) {
    case STAGE_TRACKING: {
      PoseDataPtr tracking_result = trackingOnce(curr_encoder, imus, soc_imus, mag, is_mag, ultra_measurement, get_ultra, depth_z);
      break;
    }
    case STAGE_TRACKING_RELOCALIZATION: {
      PoseDataPtr tracking_result = trackingOnce(curr_encoder, imus, soc_imus, mag, is_mag, ultra_measurement, get_ultra, depth_z);
      relocalizeOnce();
      break;
    }
    case STAGE_TRACKING_MAPPING: {
      PoseDataPtr tracking_result = trackingOnce(curr_encoder, imus, soc_imus, mag, is_mag, ultra_measurement, get_ultra, depth_z);
      mappingOnce(tracking_result);
      break;
    }
  }

//    bool output_result = getLowFrequencyPose(curr_encoder->getTime());
//    this->new_result_flag = output_result ? true : false;
  // }
}
