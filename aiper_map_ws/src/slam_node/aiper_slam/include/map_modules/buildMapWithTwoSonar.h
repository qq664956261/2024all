#pragma once

#include "arcFusion.h"
#include "utils.h"
#include "pose_data.h"
#include "interface.h"
#include "map_modules/mapPoint.h"
#include "sonarBase.h"
#include "hj_interface/Pose.h"
#include "hj_interface/Ultra.h"
#include "map_modules/icp_match.h"
#include "hj_interface/SlamActionRequest.h"
#include "hj_interface/SlamActionResponse.h"
#include "hj_interface/SlamNaviWorkResult.h"
#include "hj_interface/SlamNaviWorkResultRequest.h"
#include "hj_interface/SlamNaviWorkResultResponse.h"
#include "shm_interface.h"
#include "config_parameters.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include "map_modules/ExtrinsicErrorTerm/pointType.h"
#include "map_modules/ExtrinsicErrorTerm/icp_3d.h"
#include "map_modules/ExtrinsicErrorTerm/g2oFactor.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "g2o/solvers/dense/linear_solver_dense.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <atomic>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"



namespace HJ_slam {
namespace HJ_mapping {
class BuildMapWithTwoSonar;

typedef std::shared_ptr<BuildMapWithTwoSonar> BuildMapWithTwoSonarPtr;
typedef std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, Odom> Point_Odom;

class BuildMapWithTwoSonar {
 public: // 对外接口
  /********************************处理超声波*******************************************/
  //超声波初始化
  explicit BuildMapWithTwoSonar(const rapidjson::Value &json_conf);

  ~BuildMapWithTwoSonar();

  void reset();
  void start();
  void stop();
  void resetOptmizationData();
  void startBuildMapping();
  void stopBuildMapping();
  bool isMappingFinished();
  bool isStopSendingMsg();
  bool isMapBuildingSuccess();

  void shutdown();
  void setSonarData(const ULtraData &ultra_data);
  void setLeftFrontSonarData(const LEFTFrontData &ultra_data);
  void setLeftBackSonarData(const LEFTBackData &ultra_data);
  void setLeftTofData(const LEFTTofData &tof_data);

  void setFusionPose(const PoseDataPtr &slam_pose);
  bool loadMapBin();
  bool saveMapBin(const std::string &file_path, const std::list<MapPointPtr> &point_cloud);

  std::list<MapPointPtr> getICPTargetCloud();

  bool getIcpTransform(std::vector<Eigen::Matrix4d> &icp_trans_list);

 private: // 私有成员

  void run_map();

#ifdef X9
  void doMappingFusion(std::deque<Odom> &receive_poses, std::deque<ReceivedSonar> &receive_ultras);
#endif

#ifdef T1_pro
  void doMappingFusion(std::deque<Odom> &receive_poses, std::deque<ReceivedTof> &receive_tofs);
#endif
  /********************************处理超声波*******************************************/

  void getSonarPoint(const float &length, const SonarStatus &sonar_status, const Odom &odom, MapPointPtr &sonar_point);

  /********************************优化*******************************************/
  int loopClosureDetection();
  // 判断一个点云中的点是否大致分布在一个平面上
  bool arePointsOnLine(mypcl::PointCloud<mypcl::PointXYZI>::Ptr &cloud);
  bool icpRegistration(int id1, int id2, SE3& icp_result);
  void cluster(SynchronizedData &odom_sonar_time);
  void addG2oVertex(Odom& odom);
  void addG2oEdge();
  void addG2oLoopEdge(int id1, int id2, SE3& icp_result);
  void doLoopCloseOptimization();
  void optimizeFunc();
  void savePlyFile(int &loop_id);
  void saveOneSequencePoints(const int index);
  void removeOutlier(std::list<MapPointPtr> &target);
  void putIcpTransform(Eigen::Matrix4d &icp_transform);
  float calculateArea(const std::vector<MapPointPtr>& sonar_pose_ptr_vec_lf);
  std::mutex gthread_build_map_mutex;
  std::thread* gthread_build_map = nullptr;

  std::shared_ptr<IcpMatcher> icp_matcher_ptr_;

  std::deque<Odom> receive_poses;
#ifdef X9
  std::deque<ReceivedSonar> receive_ultras;
#endif
#ifdef T1_pro
  std::deque<ReceivedTof> receive_tofs;
#endif
  bool set_first_odom = false;
  uint64_t init_timeStamp = 0;
  Odom current_odom;

#ifdef X9
  std::mutex get_front_ultra_mutex_;
  std::mutex get_back_ultra_mutex_;
  std::deque<ReceivedSonar> inserted_front_ultras_;
  std::deque<ReceivedSonar> inserted_back_ultras_;
#endif

#ifdef T1_pro
  std::mutex get_tof_mutex_;
  std::deque<ReceivedTof> inserted_left_tofs_;
#endif

  std::mutex get_pos_mutex_;
  std::mutex target_cloud_mutex_;
  std::deque<Odom> inserted_odoms;

  // save_log
  std::string savelog_path_;
  std::string loadmap_path_;
  std::string savemap_path_;
  
  std::vector<double> ultra_para;
  MappingConfigParameters mapping_config_parameters_;
  Sonar sonar_left_front;
  Sonar sonar_left_back;

  std::unordered_map<uint64_t, MapPointPtr> gridMap;

  std::list<MapPointPtr> map_target_cloud_;
  std::atomic<bool> stop_buildMapping_;
  std::atomic<bool> stop_sendingMsg_;
  std::atomic<bool> is_building_success_;

  /******************************** loop and optimize *************************************/
  bool is_first_cluster_ = true;
  bool is_cluster_success_ = false; // 是否完成一次聚类，完成后要进行回环检测 // TODO 是否必要该变量
  bool is_optimize_success_ = false;  // 是否完成优化
  int icp_failed_num = 0;
  int vertex_id_ = 0;
  int edge_id_ = 0;
  int loop_edge_id_ = 0;
  g2o::SparseOptimizer optimizer_;
  //int loop_id_
  Point_Odom point_odom_;
  // <第i段所有数据转换到第i段第1帧base坐标系下的点的坐标的集合, <第i段第1帧所对应的pose_time, 第i段第1帧所对应的pose ：world to base> >
  std::vector<Point_Odom> point_odom_vec_;
  std::vector<Eigen::Matrix4d> matrix_orig_vec_;
  std::vector<Eigen::Matrix4d> matrix_opti_vec_;

  std::mutex icp_transform_carrier_lock_;
  std::deque<Eigen::Matrix4d> icp_transform_list;

};

} // namespace HJ_mapping
} // namespace HJ_slam