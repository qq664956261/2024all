#include <string>
#include <fstream>
#include <iostream>
#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "interface.h"
#include "pose_data.h"
#include "manager.h"
#include "hj_interface/Pose.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/MagOutput.h"
#include "hj_interface/Imu.h"
#include "hj_interface/SocImu.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Depth.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/LeftFront.h"
#include "hj_interface/LeftBack.h"
#include "hj_interface/LeftTof.h"
#include "hj_interface/NaviToSlamRotate.h"
#include "hj_interface/SlamAction.h"
#include "hj_interface/SlamNaviWorkResult.h"
#include "hj_interface/RelocalizationResult.h"
#include "log.h"
#include "map_modules/buildMapWithTwoSonar.h"
#include "aiper_slam_fusion.h"

std::string sData_path;
extern std::unique_ptr<SLAMManager> impl_;

uint16_t current_timestamp_g;

struct SequenceData {
  uint64_t ts;
  int type;
};

void ImuCallback(const hj_interface::Imu& msg) {
  IMUData imu_data_;
  imu_data_.timestamp = static_cast<uint64_t>(msg.custom_time.toSec() * 1e6);
#ifdef T1_pro
  if (static_cast<float>(msg.roll) <= 0) {
    imu_data_.roll = static_cast<float>(msg.roll) + 18000.0;
  }
  else {
    imu_data_.roll = static_cast<float>(msg.roll) - 18000.0;
  }
#endif
#ifdef X9
  imu_data_.roll = static_cast<float>(msg.roll);
#endif
  imu_data_.pitch = static_cast<float>(msg.pitch);
  imu_data_.yaw = static_cast<float>(msg.yaw);

  imu_data_.acc_x = static_cast<float>(msg.accel_x);
  imu_data_.acc_y = static_cast<float>(msg.accel_y);
  imu_data_.acc_z = static_cast<float>(msg.accel_z);
  imu_data_.gyro_x = static_cast<float>(msg.gyro_x);
  imu_data_.gyro_y = static_cast<float>(msg.gyro_y);
  imu_data_.gyro_z = static_cast<float>(msg.gyro_z);

  PutImuData(imu_data_);

  // HJ_INFO("sub imu msgs time: %.6f", msg.custom_time.toSec());
  // HJ_INFO("sub imu index: %" PRIu64, msg.index);
  // HJ_INFO("sub imu msgs machine time: : %.6f", hj_bf::HJTime::now().toSec());
}

bool slamActionCallback(hj_interface::SlamActionRequest &req, hj_interface::SlamActionResponse &res) {
  uint8_t curr_cmd = static_cast<uint8_t>(req.action_cmd);
  HJ_ERROR("slamActionCallBack =  %d", curr_cmd);
  std::vector<uint8_t> tasks = {1, 2, 11, 13, 14, 15, 26, 28, 35};
  if (std::find(tasks.begin(), tasks.end(), curr_cmd) != tasks.end()) {
    PutTaskData(curr_cmd);
  } else {
    HJ_WARN("Task > task id is not valid %d!", curr_cmd);
  }
  res.result = 0;
  return true;
}

bool relocalizationActionCallback(hj_interface::RelocalizationResultRequest &req, hj_interface::RelocalizationResultResponse &res) {
  Eigen::Matrix4d relocalization_pose = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond relocalization_quaternion(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
  relocalization_pose.block<3, 3>(0, 0) = relocalization_quaternion.toRotationMatrix();
  relocalization_pose.block<3, 1>(0, 3) = Eigen::Vector3d(req.pose.position.x, req.pose.position.y, req.pose.position.z);

  Eigen::Matrix4d water_entry_pose = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond water_entry_quaternion(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
  water_entry_pose.block<3, 3>(0, 0) = water_entry_quaternion.toRotationMatrix();
  water_entry_pose.block<3, 1>(0, 3) = Eigen::Vector3d(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  uint8_t task_id = req.task_id;

  PutRelocalizationData(relocalization_pose, water_entry_pose, req.relocalization_result, req.building_frames_result, task_id);
  HJ_INFO("relocalization result: %lf, %lf, %lf, %lf, %lf, %lf, %lf, %d, %d", 
          req.pose.position.x, req.pose.position.y, req.pose.position.z, 
          req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z,req.pose.orientation.w, 
          static_cast<int>(req.relocalization_result),
          static_cast<int>(req.building_frames_result));
  
  res.result = 0;
  return true;
}

void SocImuCallback(const hj_interface::SocImu& msg) {
  SOCIMUData soc_imu_data_;
  soc_imu_data_.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);

  soc_imu_data_.gyro_x = static_cast<float>(msg.gyro_x);
  soc_imu_data_.gyro_y = static_cast<float>(msg.gyro_y);
  soc_imu_data_.gyro_z = static_cast<float>(msg.gyro_z);
  soc_imu_data_.acc_x = static_cast<float>(msg.accel_x);
  soc_imu_data_.acc_y = static_cast<float>(msg.accel_y);
  soc_imu_data_.acc_z = static_cast<float>(msg.accel_z);

  PutSocImuData(soc_imu_data_);
  // HJ_INFO("sub soc imu msgs time: %.6f", msg.timestamp.toSec());
  // HJ_INFO("sub imu index: %" PRIu64, msg.index);
  // HJ_INFO("sub imu msgs machine time: : %.6f", hj_bf::HJTime::now().toSec());
}

void MagCallback(const hj_interface::Mag& msg) {
  MAGData mag_data_;
  mag_data_.timestamp = static_cast<uint64_t>(msg.custom_time.toSec() * 1e6);
  mag_data_.mag_x = static_cast<int16_t>(msg.mag_x);
  mag_data_.mag_y = static_cast<int16_t>(msg.mag_y);
  mag_data_.mag_z = static_cast<int16_t>(msg.mag_z);

  PutMagData(mag_data_);

  // HJ_INFO("sub mag msgs: %.6f", msg.custom_time.toSec());
  // HJ_INFO("sub mag msgs: %.6f", hj_bf::HJTime::now().toSec());
}

void EncCallback(const hj_interface::Encoder& msg) {
  ENcoderData e_data;
  e_data.exposure_ts = static_cast<uint64_t>(msg.custom_time.toSec() * 1e6);
  e_data.v_l = static_cast<double>(msg.left_msg);
  e_data.v_r = static_cast<double>(msg.right_msg);
  PutEncoderData(e_data);

  // HJ_INFO("sub enc msgs time: %.6f", msg.custom_time.toSec());
  // HJ_INFO("sub enc index: %" PRIu64, msg.index);
  // HJ_INFO("sub enc msgs machine time: %.6f", hj_bf::HJTime::now().toSec());
}

void TripleUltraCallback(const hj_interface::TripleUltra& msg) {
  ULtraData ultra_data;
  ultra_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  current_timestamp_g = ultra_data.timestamp;
  ultra_data.front_distance_l = static_cast<uint32_t>(msg.front_l);
  ultra_data.front_distance_m = static_cast<uint32_t>(msg.front_m);
  ultra_data.front_distance_r = static_cast<uint32_t>(msg.front_r);
  ultra_data.status = static_cast<uint8_t>(msg.status);
//  ultra_data.id = msg.id;
  PutUltraData(ultra_data);
  // ROS_INFO("sub enc msgs: %.6f", msg.timestamp.toSec());
}

void DepthCallback(const hj_interface::Depth& msg) {
  DEPData depth_data;
  depth_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  depth_data.pressure = static_cast<int32_t>(msg.pressure);
  depth_data.temperature = static_cast<int32_t>(msg.temp);

  PutDepthData(depth_data);
  // ROS_INFO("sub enc msgs: %.6f", msg.custom_time.toSec());
}

void NaviSlamCallback(const hj_interface::NaviToSlamRotate& msg) {
  HJ_INFO("machine has navi date");
  NAVISlamData navi_slam_data;
  navi_slam_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  navi_slam_data.mode = static_cast<uint8_t>(msg.navi_mode);
  navi_slam_data.rotation_state = static_cast<uint8_t>(msg.rotate_state);
  if (navi_slam_data.rotation_state == 1 || navi_slam_data.rotation_state == 2) {
    PutNaviSlamData(navi_slam_data);
  }

  // ROS_INFO("sub enc msgs: %.6f", msg.custom_time.toSec());
}

#ifdef X9
void LeftFrontUltraCallback(const hj_interface::LeftFront& msg) {
  LEFTFrontData left_front_data;
  left_front_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  left_front_data.distance = static_cast<uint32_t>(msg.dist);
  left_front_data.status = static_cast<uint8_t>(msg.status);

  PutLeftFrontUltraData(left_front_data);

}

void LeftBackUltraCallback(const hj_interface::LeftBack& msg) {
  LEFTBackData left_back_data;
  left_back_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  left_back_data.distance = static_cast<uint32_t>(msg.dist);
  left_back_data.status = static_cast<uint8_t>(msg.status);

  PutLeftBackUltraData(left_back_data);

}
#endif

#ifdef T1_pro
void LeftTofCallback(const hj_interface::LeftTof& msg) {
  LEFTTofData left_tof_data;
  left_tof_data.timestamp = static_cast<uint64_t>(msg.timestamp.toSec() * 1e6);
  left_tof_data.front_distance = static_cast<uint32_t>(msg.dist_front);
  left_tof_data.back_distance = static_cast<uint32_t>(msg.dist_back);
  PutLeftTofData(left_tof_data);
}
#endif

void readEncoderData(std::string &encoder_txt, ENcoderData &e_data) {
  std::ifstream ifs;
  ifs.open(encoder_txt, std::ios::in);
  if (!ifs.is_open())
  {
    std::cout << "failure to read encoder data " << encoder_txt<<std::endl;
    return; 
  }
  double ts_{0.0f};//timeStamps
  double v_l{0.0f}, v_r{0.0f};

  while (ifs >> ts_ && ifs >> v_l && ifs >> v_r)
  {
    e_data.exposure_ts = static_cast<uint64_t>(ts_ * 1e6);
    e_data.v_l = static_cast<float>(v_l);
    e_data.v_r = static_cast<float>(v_r);
  }
  usleep(8000);
  ifs.close();
}

void ReadImuDataPimaxFormat(const std::string &file, std::queue<IMUData> &imu_data) {
  std::ifstream imu_data_ifs;
  imu_data_ifs.open(file.c_str());
  if (imu_data_ifs) {
    Eigen::Matrix<double, 10, 1> data = Eigen::Matrix<double, 10, 1>::Zero();
    double timestamp{0.0f};
    while (imu_data_ifs >> timestamp >> data[1] >> data[2] >> data[3]\
    >> data[4] >> data[5] >> data[6]>> data[7] >> data[8] >> data[9]) {
    //todo check imu data valid
    IMUData imu_data_;
    // ms
    imu_data_.timestamp = static_cast<uint64_t >(timestamp * 1e6);
    imu_data_.roll = static_cast<float>(data[1]);
    imu_data_.pitch = static_cast<float>(data[2]);
    imu_data_.yaw = static_cast<float>(data[3]);
    imu_data_.acc_x = static_cast<float>(data[4]);
    imu_data_.acc_y = static_cast<float>(data[5]);
    imu_data_.acc_z = static_cast<float>(data[6]);
    imu_data_.gyro_x = static_cast<float>(data[7]);
    imu_data_.gyro_y = static_cast<float>(data[8]);
    imu_data_.gyro_z = static_cast<float>(data[9]);
    imu_data.push(imu_data_);

    }
  }
}

void PubIMUData() {
  std::cout << "here in pub imu data" << std::endl;
  std::string imu_path = sData_path + "/Data/imu.txt";
  std::queue<IMUData> imu_data;
  ReadImuDataPimaxFormat(imu_path, imu_data);
  std::cout<<"read imu is ending"<<std::endl;
  std::ifstream m_ofs_seq;
  std::string sequences_path = sData_path + "/Data/sequences.txt";
  m_ofs_seq.open(sequences_path.c_str());
  if (!m_ofs_seq) {
    printf("failed to open file sequences.txt\n");
    return;
  } else {
    uint64_t ts{0};
    int type{0};
    while (m_ofs_seq >> type >> ts) {
      if (type == 23) {
      } else if (type == 0) {
        IMUData data = imu_data.front();
        IMUData imu_data_;
        imu_data_.timestamp = data.timestamp;
        imu_data_.roll = data.roll;
        imu_data_.pitch = data.pitch;
        imu_data_.yaw = data.yaw;
        imu_data_.acc_x = data.acc_x;
        imu_data_.acc_y = data.acc_y;
        imu_data_.acc_z = data.acc_z;
        imu_data_.gyro_x = data.gyro_x;
        imu_data_.gyro_y = data.gyro_y;
        imu_data_.gyro_z = data.gyro_z;

        PutImuData(imu_data_);
        imu_data.pop();
      }
    }
    std::cout << "end publish imu data" << std::endl;
  }
}

void PubEncoderData() {
  std::cout << "here in pub encoder data" << std::endl;

  std::string imu_path = sData_path + "Data/imu.txt";
  std::queue<IMUData> imu_data;
  ReadImuDataPimaxFormat(imu_path, imu_data);

  std::ifstream m_ofs_seq;
  std::string sequences_path = sData_path + "/Data/sequences.txt";

  m_ofs_seq.open(sequences_path.c_str());
  if (!m_ofs_seq) {
    printf("failed to open file sequences.txt\n");
    return;
  } else {
    std::queue<SequenceData> sequence_datas;
    uint64_t ts{0};
    int type{0};
    while (m_ofs_seq >> type >> ts) {
      SequenceData sequence_data;
      sequence_data.type = type;
      sequence_data.ts = ts;
      sequence_datas.push(sequence_data);
    }

    while (sequence_datas.size() > 1) {
      // static int encoder_cnt = 0;
      SequenceData sequence_data_ = sequence_datas.front();
      int type_ = sequence_data_.type;
      uint64_t ts_ = sequence_data_.ts;
      if (type_ == 23) {
        // encoder_cnt++;
        std::string file_name = std::to_string(ts_) + ".txt";
        //  read encoder data from txt
        std::string encoder_Path = sData_path;
        encoder_Path.append("/Data/encoder/");
        encoder_Path.append(file_name);
        ENcoderData e_data;
        readEncoderData(encoder_Path, e_data);
        PutEncoderData(e_data);
      } else if (type_ == 0) {
        }
      sequence_datas.pop();
    }
    std::cout << "end publish image data" << std::endl;
  }
}

void toRosPontCloud(const std::list<MapPointPtr> &src_point_cloud, sensor_msgs::PointCloud2 &cloud_msg){
  if(src_point_cloud.empty()) {
    HJ_INFO("Source point cloud is empty.");
    return;
  }
  cloud_msg.header.frame_id = "odom";
  cloud_msg.header.stamp = ros::Time().fromSec(static_cast<double>(current_timestamp_g) / 1e6);
  cloud_msg.width = src_point_cloud.size(); // 设置点云宽度
  cloud_msg.height = 1; // 设置点云高度
  cloud_msg.is_dense = 0u; // 设置点云密集性

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(src_point_cloud.size()); // 点云中有1个点
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (const auto& map_point : src_point_cloud) {
    if (map_point == nullptr) {
      continue;
    }
    *iter_x = map_point->getCor_x();
    *iter_y = map_point->getCor_y();
    *iter_z = map_point->getCor_z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

void safeTimeFromSec(uint64_t t_64, double t) {
  if (t < 0 || t > std::numeric_limits<uint32_t>::max()) {
    HJ_INFO("Time int : %ld is out of dual 32-bit range", t_64);
    HJ_ERROR("Time int: %ld is out of dual 32-bit range", t_64);
    HJ_INFO("Time double : %lf is out of dual 32-bit range", t);
    HJ_ERROR("Time double: %lf is out of dual 32-bit range", t);
//    throw std::out_of_range("Time is out of dual 32-bit range");
  }
}

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<aiper_slam_fusion_ns::AiperSlam>(FUNCTION_NAME);
}

namespace aiper_slam_fusion_ns {

void AiperSlam::pubPoseCallback(const hj_bf::HJTimerEvent &) {
  std::unique_lock<std::mutex> lock(impl_->output_lock_);
  while (!impl_->output_pose_queue.empty()) {
    PoseDataPtr fusion_result = impl_->output_pose_queue.front();
    impl_->output_pose_queue.pop();
    lock.unlock();
    double curr_time = static_cast<double>(fusion_result->time_) / 1e6;

    hj_interface::Pose pose_msg;
    Eigen::Quaterniond q = fusion_result->q_;
    Eigen::Vector3d p = fusion_result->p_;
    Eigen::Vector3d euler = fusion_result->euler_;
    safeTimeFromSec(fusion_result->time_, curr_time);
    pose_msg.timestamp = ros::Time().fromSec(curr_time);
    pose_msg.x = p[0];
    pose_msg.y = p[1];
    pose_msg.z = p[2];
    // pose_msg.pose.pose.orientation.x = q.x();
    // pose_msg.pose.pose.orientation.y = q.y();
    // pose_msg.pose.pose.orientation.z = q.z();
    // pose_msg.pose.pose.orientation.w = q.w();
    pose_msg.roll = euler[0];
    pose_msg.pitch = euler[1];
    pose_msg.yaw = euler[2];
    pose_msg.type = static_cast<uint8_t>(fusion_result->type_);
    pose_pub_.publish(pose_msg);
    impl_->new_result_flag = false;
    //HJ_INFO("#pub pose navi: %f, %f, %f, %f, %f, %f, %f", pose_msg.roll, pose_msg.pitch, pose_msg.yaw, pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z, curr_time);

#ifdef DEMO_SHOW
//    geometry_msgs::PoseStamped pose_tmp;
//    pose_tmp.pose.position.x = p[0];
//    pose_tmp.pose.position.y = p[1];
//    pose_tmp.pose.position.z = p[2];
//    pose_tmp.pose.orientation.x = q.x();
//    pose_tmp.pose.orientation.y = q.y();
//    pose_tmp.pose.orientation.z = q.z();
//    pose_tmp.pose.orientation.w = q.w();
//    path_.header.stamp = ros::Time().fromSec(curr_time);
//    path_.poses.push_back(pose_tmp);
//    path_pub_.publish(path_);

    nav_msgs::Odometry odom_msg;
    safeTimeFromSec(fusion_result->time_, curr_time);
    odom_msg.header.stamp = ros::Time().fromSec(curr_time);
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = p[0];
    odom_msg.pose.pose.position.y = p[1];
    odom_msg.pose.pose.position.z = p[2];
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_.publish(odom_msg);
#endif
    lock.lock();
  }

}

void AiperSlam::pubMagAngleCallback(const hj_bf::HJTimerEvent &) {
  std::unique_lock<std::mutex> lock(impl_->output_mag_lock_);
  while (!impl_->output_mag_angle_queue.empty()) {
    MagOutputDataPtr mag_angle_result = impl_->output_mag_angle_queue.front();
    impl_->output_mag_angle_queue.pop();
    lock.unlock();
    double curr_time = static_cast<double>(mag_angle_result->time_) / 1e6;
    hj_interface::MagOutput mag_angle_msg;
    mag_angle_msg.timestamp = ros::Time().fromSec(curr_time);
    mag_angle_msg.yaw = mag_angle_result->yaw_;
    mag_angle_pub_.publish(mag_angle_msg);
    HJ_INFO("#pub mag estimate angle navi: %f, %f", mag_angle_result->yaw_, curr_time);
    lock.lock();
  }

}

void AiperSlam::pubPointCloudCallback(const hj_bf::HJTimerEvent &){

  // SLAMManager::Stage curr_stage = impl_->getStage();
  if (impl_->isInitSuccess() == true) {
    // std::list<MapPointPtr> src_point_cloud = impl_->getICPSourceCloud();
    std::list<MapPointPtr> target_point_cloud = impl_->getICPTargetCloud();

//    sensor_msgs::PointCloud2 source_cloud_msg;
    sensor_msgs::PointCloud2 target_cloud_msg;
//    toRosPontCloud(src_point_cloud, source_cloud_msg);
//    source_cloud_pub_.publish(source_cloud_msg);
    toRosPontCloud(target_point_cloud, target_cloud_msg);
    target_cloud_pub_.publish(target_cloud_msg);
  }
}

AiperSlam::AiperSlam(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  if (json_conf.HasMember("aiper_slam_work_dir") && json_conf["aiper_slam_work_dir"].IsString()) {
    sData_path = json_conf["aiper_slam_work_dir"].GetString();
    HJ_INFO("sData_path: %s", sData_path.c_str());
  }
  else {
    HJ_ERROR("sData_path: %s", sData_path.c_str());
    HJ_ERROR("warnning: can not load slam json file !!!");
  }

  std::string relocalization_dir = "";
  if (json_conf.HasMember("aiper_relocalization_dir") && json_conf["aiper_relocalization_dir"].IsString()) {
    relocalization_dir = json_conf["aiper_relocalization_dir"].GetString();
    HJ_INFO("relocalization directory: %s", relocalization_dir.c_str());
  }
  else {
    HJ_ERROR("warnning: can not load relocalization directory from json file %s !!!", relocalization_dir.c_str());
  }
  double time_diff = 0.0;
#ifdef DEMO_SHOW
  time_diff = 1000;//以1秒一次的频率对外发布点云
#else
  time_diff = 30;
#endif

  std::string output_path = sData_path;
  std::string config_path = sData_path;
  std::string voc_file = " ";

  // pubulish subscirbe init
  imu_sub_ = hj_bf::HJSubscribe("/imu_chatter", 1000, &ImuCallback);
  soc_imu_sub_ = hj_bf::HJSubscribe("/soc_imu_chatter", 1000, &SocImuCallback);
  mag_sub_ = hj_bf::HJSubscribe("/mag_chatter", 1000, &MagCallback);
  enc_sub_ = hj_bf::HJSubscribe("/motor_chatter", 1, &EncCallback);
  depth_sub_ = hj_bf::HJSubscribe("/depth_chatter", 1000, &DepthCallback);
  navi_slam_sub_ = hj_bf::HJSubscribe("/navi_slam_rotate_state", 1000, &NaviSlamCallback);

  triple_ultra_sub_ = hj_bf::HJSubscribe("/triple_ultra", 1000, &TripleUltraCallback);
#ifdef X9
  left_front_ultra_sub_ = hj_bf::HJSubscribe("/x9/left_front", 1000, &LeftFrontUltraCallback);
  left_back_ultra_sub_ = hj_bf::HJSubscribe("/x9/left_back", 1000, &LeftBackUltraCallback);
#endif

#ifdef T1_pro
  left_tof_sub_ = hj_bf::HJSubscribe("/t1pro/left_tof", 1000, &LeftTofCallback);
#endif

  slam_server_ = hj_bf::HJCreateServer("/slam_action_service", &slamActionCallback);
  relocalization_server_ = hj_bf::HJCreateServer("/relocalization_result_service", &relocalizationActionCallback);

  SensorFusionInitialImpl(config_path, output_path, relocalization_dir, voc_file);

  pose_pub_ = hj_bf::HJAdvertise<hj_interface::Pose>("/fusion_result", 1);
  mag_angle_pub_ = hj_bf::HJAdvertise<hj_interface::MagOutput>("/mag_angle_result", 1);
//  source_cloud_pub_ = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/source_cloud", 1);
  target_cloud_pub_ = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
  odom_pub_ = hj_bf::HJAdvertise<nav_msgs::Odometry>("/odom", 1);
  path_pub_ = hj_bf::HJAdvertise<nav_msgs::Path>("/path", 1);
  path_.header.frame_id = "odom";

//  SensorFusionInitialImpl(config_path_, output_path, voc_file);
  mag_angle_pub_timer = hj_bf::HJCreateTimer("mag_angle_pub_timer", 15 * 1000, &AiperSlam::pubMagAngleCallback, this);
  pose_pub_timer_ = hj_bf::HJCreateTimer("pose_pub_timer", 15 * 1000, &AiperSlam::pubPoseCallback, this);
#ifdef DEMO_SHOW
  cloud_pub_timer = hj_bf::HJCreateTimer("point_cloud_pub_timer", time_diff * 1000, &AiperSlam::pubPointCloudCallback, this);
#endif

}
}  // namespace aiper_slam_fusion_ns