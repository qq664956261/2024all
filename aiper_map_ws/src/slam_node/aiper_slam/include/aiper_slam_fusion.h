// @file AiperSlam.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#ifndef INCLUDE_AiperSlam_H//your macro
#define INCLUDE_AiperSlam_H

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

#include "hj_interface/Pose.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Depth.h"
#include "hj_interface/Ultra.h"
#include "function_factory.h"
#include "node_factory.h"

#include "interface.h"
#include "pose_data.h"
#include "manager.h"
#include "log.h"
#include "map_modules/buildMapWithTwoSonar.h"

namespace aiper_slam_fusion_ns {//your namespace

class AiperSlam : public hj_bf::Function {
 public:
  explicit AiperSlam(const rapidjson::Value &json_conf);
  ~AiperSlam(){};

 private:
  void pubPoseCallback(const hj_bf::HJTimerEvent &);
  void pubMagAngleCallback(const hj_bf::HJTimerEvent &);
  void pubPointCloudCallback(const hj_bf::HJTimerEvent &);

  ros::NodeHandle nh_;
  hj_bf::HJSubscriber imu_sub_;
  hj_bf::HJSubscriber soc_imu_sub_;
  hj_bf::HJSubscriber mag_sub_;
  hj_bf::HJSubscriber enc_sub_;
  hj_bf::HJSubscriber triple_ultra_sub_;
  hj_bf::HJSubscriber depth_sub_;
  hj_bf::HJSubscriber navi_slam_sub_;
#ifdef X9
  hj_bf::HJSubscriber left_front_ultra_sub_;
  hj_bf::HJSubscriber left_back_ultra_sub_;
#endif
#ifdef T1_pro
  hj_bf::HJSubscriber left_tof_sub_;
#endif
  hj_bf::HJPublisher pose_pub_;
  hj_bf::HJPublisher odom_pub_;
  hj_bf::HJPublisher mag_angle_pub_;
  hj_bf::HJPublisher source_cloud_pub_;
  hj_bf::HJPublisher path_pub_;
  hj_bf::HJPublisher target_cloud_pub_;
  hj_bf::HJTimer pose_pub_timer_, cloud_pub_timer, mag_angle_pub_timer;
  hj_bf::HJServer slam_server_;
  hj_bf::HJServer relocalization_server_;

  nav_msgs::Path path_;
};
}  // namespace aiper_slam_fusion_ns

#endif


