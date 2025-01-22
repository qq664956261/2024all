#include "monte_particle_filter.h"
#include "map_modules/buildMapWithTwoSonar.h"
#include "map_modules/icp_match.h"
#include "logic_dev/shm_data.h"
#include "common_tool.h"

Eigen::Vector3d map_t_bm_x9 = Eigen::Vector3d(-0.11, 0.0, 0.0);
Eigen::Vector3d map_t_bm_t1_pro = Eigen::Vector3d(0.0, 0.0, 0.0);

namespace {
void exportMapToAPP(const std::list<MapPointPtr> &map_point_list, const float groundArea) {
  int map_point_size = static_cast<int>(map_point_list.size());
  if(map_point_size > shm_data::MAX_POINT - 10) {
    HJ_ERROR("map size is too large, please increase MAX_POINT, map size: %zu", map_point_list.size());
    return;
  }
  shm_data::MapData map_data[shm_data::MAX_POINT] = {};
  map_data[0].x = static_cast<int32_t>(map_point_list.size());
  map_data[0].y = 1;

#ifdef X9
  map_data[1].x = static_cast<int32_t>(map_t_bm_x9[0] * 1000.0 + 0.5);
  map_data[1].y = static_cast<int32_t>(map_t_bm_x9[1] * 1000.0 + 0.5);
  map_data[1].z = static_cast<int32_t>(map_t_bm_x9[2] * 1000.0 + 0.5);
#endif
#ifdef T1_pro
  map_data[1].x = static_cast<int32_t>(map_t_bm_t1_pro[0] * 1000.0 + 0.5);
  map_data[1].y = static_cast<int32_t>(map_t_bm_t1_pro[1] * 1000.0 + 0.5);
  map_data[1].z = static_cast<int32_t>(map_t_bm_t1_pro[2] * 1000.0 + 0.5);
#endif

  map_data[2].x = static_cast<int32_t>(groundArea * 10);
  HJ_INFO("export ground area = %d", map_data[2].x);
  int i = 10;
  for (const auto& it_cloud : map_point_list) {
    if (it_cloud == nullptr) {
      continue;
    }
    map_data[i].x = static_cast<int32_t>(it_cloud->getCor_x() * 1000.0 + 0.5);
    map_data[i].y = static_cast<int32_t>(it_cloud->getCor_y() * 1000.0 + 0.5);
    map_data[i].z = static_cast<int32_t>(it_cloud->getCor_z() * 1000.0 + 0.5);
    i++;
  }

  hj_bf::setVariable("map_data_shm", map_data);
  HJ_INFO("set map varible to share memory, map size: %d", static_cast<int>(map_data[0].x));
}

void getMapFromAPPTest() {
  shm_data::MapData map_data_temp[shm_data::MAX_POINT] = {};
  bool ret = hj_bf::getVariable("map_data_shm", map_data_temp);
  if (ret) {
    HJ_INFO("get map varible from share memory, map size: %d", map_data_temp[0].x);
  }
  else {
    HJ_ERROR("get map varible from share memory failed");
  }
}
} // namespace

namespace HJ_slam {
namespace HJ_mapping {

class ErrorStateKalmanFilter;
std::shared_ptr<ErrorStateKalmanFilter> eskf_ptr_;

static std::ofstream mofDataLogIncc;
static std::ofstream mofLogoutIncc;
hj_bf::HJClient slam_clinet = hj_bf::HJCreateClient<hj_interface::SlamNaviWorkResult>("/slam_navi_action_result_service");

static void exportMapSuccessInfo() {
//  hj_bf::HJClient slam_clinet = hj_bf::HJCreateClient<hj_interface::SlamNaviWorkResult>("/slam_navi_action_result_service");
  hj_interface::SlamNaviWorkResult curr_task;
  curr_task.request.action_cmd = 1;
  curr_task.request.action_result = 1;
  if(slam_clinet.call(curr_task)) {
    HJ_INFO("client building map successed is successfully published");
  }
  else{
    HJ_INFO("client building map successed is failed published");
  }

}

static void exportMapFailInfo() {
//  hj_bf::HJClient slam_clinet = hj_bf::HJCreateClient<hj_interface::SlamNaviWorkResult>("/slam_navi_action_result_service");
  hj_interface::SlamNaviWorkResult curr_task;
  curr_task.request.action_cmd = 1;
  curr_task.request.action_result = 2;
  if(slam_clinet.call(curr_task)) {
    HJ_INFO("client building map failed is successfully published");
  }
  else{
    HJ_INFO("client building map failed is failed published");
  }

}

void BuildMapWithTwoSonar::resetOptmizationData() {
  point_odom_.first->points.clear();
  point_odom_.first = nullptr;
  point_odom_vec_.clear();
  matrix_orig_vec_.clear();
  matrix_opti_vec_.clear();
  {
    std::unique_lock<std::mutex> lck(icp_transform_carrier_lock_);
    icp_transform_list.clear();
  }
}

BuildMapWithTwoSonar::~BuildMapWithTwoSonar() {
  mofLogoutIncc << __FUNCTION__ << " " << ros::Time::now() << std::endl;
  HJ_ERROR("class BuildMapWithTwoSonar is quiting");
  stop();

  receive_poses.clear();
#ifdef X9
  receive_ultras.clear();

  {
    std::unique_lock<std::mutex> lock(get_front_ultra_mutex_);
    inserted_front_ultras_.clear();
  }
  {
    std::unique_lock<std::mutex> lock(get_back_ultra_mutex_);
    inserted_back_ultras_.clear();
  }
#endif

#ifdef T1_pro
  receive_tofs.clear();

{
std::unique_lock<std::mutex> lock(get_tof_mutex_);
receive_tofs.insert(receive_tofs.end(), inserted_left_tofs_.begin(), inserted_left_tofs_.end());
inserted_left_tofs_.clear();
}
#endif

  {
    std::unique_lock<std::mutex> lock(get_pos_mutex_);
    inserted_odoms.clear();
  }

  gridMap.clear();

  {
    std::unique_lock<std::mutex> lock(target_cloud_mutex_);
    map_target_cloud_.clear();
  }
  resetOptmizationData();
  mofLogoutIncc.close();
  mofDataLogIncc.close();
}


void BuildMapWithTwoSonar::start() {
  std::lock_guard<std::mutex> lock(gthread_build_map_mutex);
  if (gthread_build_map == nullptr) {
    gthread_build_map = new std::thread(&BuildMapWithTwoSonar::run_map, this);
  }
}

void BuildMapWithTwoSonar::stop() {
  stopBuildMapping();
  std::lock_guard<std::mutex> lock(gthread_build_map_mutex);
  if (gthread_build_map != nullptr) {
    if (gthread_build_map->joinable()) {
      gthread_build_map->join();
    }
    delete gthread_build_map;
    gthread_build_map = nullptr;
  }
}

void BuildMapWithTwoSonar::stopBuildMapping() {
  stop_buildMapping_.store(true);
}

BuildMapWithTwoSonar::BuildMapWithTwoSonar(const rapidjson::Value &json_conf) : stop_buildMapping_(false), stop_sendingMsg_(false), is_building_success_(false) {
  mapping_config_parameters_.loadParameters(json_conf["mapping"]);
  // TODO(edan): save map and load map flag control.
  savelog_path_ = mapping_config_parameters_.savelog_path;
  loadmap_path_ = mapping_config_parameters_.load_map_path;
  savemap_path_ = mapping_config_parameters_.save_map_path;

  ultra_para = {mapping_config_parameters_.sonar_left_front_fov_rad,
                mapping_config_parameters_.sonar_left_front_base_x,
                mapping_config_parameters_.sonar_left_front_base_y,
                mapping_config_parameters_.sonar_left_front_base_yaw,
                mapping_config_parameters_.sonar_left_back_fov_rad,
                mapping_config_parameters_.sonar_left_back_base_x,
                mapping_config_parameters_.sonar_left_back_base_y,
                mapping_config_parameters_.sonar_left_back_base_yaw};

  mofLogoutIncc.open(savelog_path_ + "/maplog.txt");
//  mofDataLogIncc.open(savelogpath + "/mapdata.txt");
  mofLogoutIncc << __FUNCTION__ << " " << ros::Time::now() << std::endl;

//  sonar_left_front.fov_rad = ultra_para[0];          // sonar1坐标系相对base_link坐标系的x外参
  sonar_left_front.sonar_base_x = ultra_para[1];  // sonar1坐标系相对base_link坐标系的y外参
  sonar_left_front.sonar_base_y = ultra_para[2];  // sonar1坐标系相对base_link坐标系的y外参
  sonar_left_front.sonar_base_yaw = ultra_para[3];
//  sonar_left_back.fov_rad = ultra_para[4];
  sonar_left_back.sonar_base_x = ultra_para[5];  // sonar2坐标系相对base_link坐标系的x外参
  sonar_left_back.sonar_base_y = ultra_para[6];   // sonar2坐标系相对base_link坐标系的x外参
  sonar_left_back.sonar_base_yaw = ultra_para[7]; // sonar2坐标系相对base_link坐标系的yaw外参

  gthread_build_map = new std::thread(&BuildMapWithTwoSonar::run_map, this);

  // icp test
//  icp_matcher_ptr_ = std::make_shared<IcpMatcher>(json_conf["icp"]);

  point_odom_.first = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

}


void BuildMapWithTwoSonar::reset() {
  mofLogoutIncc << __FUNCTION__ << " " << ros::Time::now() << std::endl;
  HJ_INFO("Reset Function BuildMapWithTwoSonar");
  stop_sendingMsg_ = false;
  is_building_success_ = false;

  receive_poses.clear();
#ifdef X9
  receive_ultras.clear();

  {
    std::unique_lock<std::mutex> lock(get_front_ultra_mutex_);
    inserted_front_ultras_.clear();
  }
  {
    std::unique_lock<std::mutex> lock(get_back_ultra_mutex_);
    inserted_back_ultras_.clear();
  }
#endif

#ifdef T1_pro
  receive_tofs.clear();

  {
  std::unique_lock<std::mutex> lock(get_tof_mutex_);
  receive_tofs.insert(receive_tofs.end(), inserted_left_tofs_.begin(), inserted_left_tofs_.end());
  inserted_left_tofs_.clear();
  }
#endif

  {
    std::unique_lock<std::mutex> lock(get_pos_mutex_);
    inserted_odoms.clear();
  }

  gridMap.clear();

  {
    std::unique_lock<std::mutex> lock(target_cloud_mutex_);
    map_target_cloud_.clear();
  }

  point_odom_.first->points.clear();
  point_odom_vec_.clear();
  matrix_orig_vec_.clear();
  matrix_opti_vec_.clear();
  {
    std::unique_lock<std::mutex> lck(icp_transform_carrier_lock_);
    icp_transform_list.clear();
  }

  is_first_cluster_ = true;
  is_cluster_success_ = false;
  is_optimize_success_ = false;
  vertex_id_ = 0;
  edge_id_ = 0;

}


#ifdef X9
void BuildMapWithTwoSonar::setLeftFrontSonarData(const LEFTFrontData &ultra_data) {
  std::unique_lock<std::mutex> lock(get_front_ultra_mutex_);
  ReceivedSonar sonarData;
  sonarData.timestamp = ultra_data.timestamp;//us
  sonarData.distance = (ultra_data.distance < 15000) ? static_cast<float>(ultra_data.distance * 0.001) : -1.0f;
  sonarData.status = SonarStatus::front;
  inserted_front_ultras_.push_back(sonarData);
  if (inserted_front_ultras_.size() > 500) {
    inserted_front_ultras_.pop_front();
  }
}

void BuildMapWithTwoSonar::setLeftBackSonarData(const LEFTBackData &ultra_data) {
  std::unique_lock<std::mutex> lock(get_back_ultra_mutex_);
  ReceivedSonar sonarData;
  sonarData.timestamp = ultra_data.timestamp;//us
  sonarData.distance = (ultra_data.distance < 15000) ? static_cast<float>(ultra_data.distance * 0.001) : -1.0f;
  sonarData.status = SonarStatus::back;
  inserted_back_ultras_.push_back(sonarData);
  if (inserted_back_ultras_.size() > 500) {
    inserted_back_ultras_.pop_front();
  }
}
#endif

#ifdef T1_pro
void BuildMapWithTwoSonar::setLeftTofData(const LEFTTofData &tof_data) {
  std::unique_lock<std::mutex> lock(get_tof_mutex_);
  ReceivedTof tofData;
  tofData.timestamp = tof_data.timestamp;
  tofData.front_distance = (tof_data.front_distance < 500) ? static_cast<float>(tof_data.front_distance * 0.001) : -1.0f;
  tofData.back_distance = (tof_data.back_distance < 500) ? static_cast<float>(tof_data.back_distance * 0.001) : -1.0f;;
  inserted_left_tofs_.push_back(tofData);
  if (inserted_left_tofs_.size() > 500) {
    inserted_left_tofs_.pop_front();
  }
}
#endif

void BuildMapWithTwoSonar::setFusionPose(const PoseDataPtr &slam_pose) {
  std::unique_lock<std::mutex> lock(get_pos_mutex_);
  Odom currPos;
  currPos.timestmap = static_cast<uint64_t>(slam_pose->time_);//us
  currPos.x = slam_pose->p_[0];
  currPos.y = slam_pose->p_[1];
  currPos.z = slam_pose->p_[2];
  currPos.yaw = slam_pose->euler_[2];
  inserted_odoms.push_back(currPos);
  if (inserted_odoms.size() > 30) { //?
    inserted_odoms.pop_front();
  }
}

void BuildMapWithTwoSonar::cluster(SynchronizedData &odom_sonar_time) {
  //logout << "cluster begin" << std::endl;
  float distance_threshold = 2.0;

  // point_map to point_base
  mypcl::PointXYZI point_map;  // map系下的点
  mypcl::PointXYZI point_base; // base系下的点
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();  // 每段第一帧在map系下的位姿
  point_map.x = odom_sonar_time.map_point.getCor_x();
  point_map.y = odom_sonar_time.map_point.getCor_y();
  point_map.z = odom_sonar_time.map_point.getCor_z();
  point_map.intensity = 0;

  if(is_first_cluster_) {
    Odom odom = odom_sonar_time.odom;
    T = odom.toEigenIsometry<Eigen::Isometry3f>();
    point_base = point_map.transform(T.inverse());
    point_odom_.first->addPoint(point_base);
    point_odom_.second = odom_sonar_time.odom;
    is_first_cluster_ = false;
    return;
  }

  // 计算当前帧与第一帧的距离，判断是否将其加入该队列
  float distance = odom_sonar_time.odom.calDistance(point_odom_.second);
  if(distance < distance_threshold) {
    Odom odom = point_odom_.second;
    T = odom.toEigenIsometry<Eigen::Isometry3f>();
    point_base = point_map.transform(T.inverse());
    point_odom_.first->addPoint(point_base);
  }
  else {
    Point_Odom pc_odom_temp;
    pc_odom_temp.first = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::copyPointCloud(*(point_odom_.first), *(pc_odom_temp.first));
    pc_odom_temp.second = point_odom_.second;
    point_odom_vec_.push_back(pc_odom_temp);
    point_odom_.first->points.clear();
    Odom odom = odom_sonar_time.odom;
    T = odom.toEigenIsometry<Eigen::Isometry3f>();
    point_base = point_map.transform(T.inverse());
    point_odom_.first->addPoint(point_base);
    point_odom_.second = odom_sonar_time.odom;
    is_cluster_success_ = true;
  }
}


std::list<MapPointPtr> BuildMapWithTwoSonar::getICPTargetCloud() {
  std::unique_lock<std::mutex> lock(target_cloud_mutex_);
  return map_target_cloud_;
}

void BuildMapWithTwoSonar::getSonarPoint(const float &length, const SonarStatus &sonar_status,
                                             const Odom &odom, MapPointPtr &sonar_point) {
  // 获取外参
  float sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;

  if (sonar_status == SonarStatus::front) {
    sonar_base_x = sonar_left_front.sonar_base_x;
    sonar_base_y = sonar_left_front.sonar_base_y;
    sonar_base_yaw = sonar_left_front.sonar_base_yaw;
  } else if (sonar_status == SonarStatus::back) {
    sonar_base_x = sonar_left_back.sonar_base_x;
    sonar_base_y = sonar_left_back.sonar_base_y;
    sonar_base_yaw = sonar_left_back.sonar_base_yaw;
  }

  float theta_rad = 0.0;
//  float sonar_z = 0.0;
  float sonar_x = length * cos(theta_rad);
  float sonar_y = length * sin(theta_rad);

  // sonar坐标系转base_link，即base系下的超声波数据
  float base_x = sonar_base_x + (sonar_x * cos(sonar_base_yaw) - sonar_y * sin(sonar_base_yaw));
  float base_y = sonar_base_y + (sonar_x * sin(sonar_base_yaw) + sonar_y * cos(sonar_base_yaw));

  // base_link转odom坐标系，即odom系下的超声波数据
  float x = odom.x + (base_x * cos(odom.yaw) - base_y * sin(odom.yaw));
  float y = odom.y + (base_x * sin(odom.yaw) + base_y * cos(odom.yaw));
  float z = odom.z;
  Eigen::Isometry3f odom_iso = odom.toEigenIsometry<Eigen::Isometry3f>();
  Eigen::Vector3f pose_base(base_x, base_y, 0);
  Eigen::Vector3f pose_map;
  pose_map = odom_iso * pose_base;
  sonar_point = std::make_shared<MapPoint>(x, y, z);

}

static bool compareByDsitance(const std::pair<int, float> &a, const std::pair<int, float> &b) {
  return a.second < b.second;
}

int BuildMapWithTwoSonar::loopClosureDetection() {
  Point_Odom pc_time_pose = point_odom_vec_.back();
  Odom curr_odom = pc_time_pose.second;
  double delta_time = static_cast<double>(curr_odom.timestmap - init_timeStamp) * 1e-6;
#ifdef With_mag
  const double loop_interval = 300.0;
#else
  const double loop_interval = 30.0;
#endif

  if (delta_time < loop_interval) {
    return -1;
  }

  std::list<std::pair<int, float>> index_distance;
  index_distance.clear();
  for(int i = 0; i < static_cast<int>(point_odom_vec_.size()) - 1; i++) {
    Odom history_odom = point_odom_vec_[i].second;
    float distance = curr_odom.calDistance(history_odom);
    index_distance.push_back(std::make_pair(i, distance));
  }
  if (index_distance.size() < 2) {
    return -1;
  }
  index_distance.sort(compareByDsitance);
  int min_id = index_distance.front().first;
  float min_distance = index_distance.front().second;

  if(min_distance < 1.0) {
    HJ_INFO("[optimize] min loop id successfully find, value: %lf", min_distance);
    return min_id;
  }
  else {
    HJ_INFO("[optimize] min loop id failed find, value: %lf", min_distance);
    return -1;
  }

}

bool BuildMapWithTwoSonar::arePointsOnLine(mypcl::PointCloud<mypcl::PointXYZI>::Ptr &cloud) {
  if(cloud->size() < 20) {
    HJ_ERROR("the size of point cloud is too small!");
    return false;
  }
  mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_judge(new mypcl::PointCloud<mypcl::PointXYZI>);
  mypcl::copyPointCloud(*cloud, *cloud_judge);
  Eigen::Matrix<double, 4, -1> neighbors(4, cloud_judge->points.size());
  for (int i = 0; i < static_cast<int>(cloud_judge->points.size()); i++) {
    Eigen::Vector4d p;
    p[0] = cloud_judge->points[i].x;
    p[1] = cloud_judge->points[i].y;
    p[2] = cloud_judge->points[i].z;
    p[3] = 1.0;
    neighbors.col(i) = p;
  }
  neighbors.colwise() -= neighbors.rowwise().mean().eval();
  Eigen::Matrix4d cov = neighbors * neighbors.transpose() / cloud_judge->points.size();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d values;
  values = svd.singularValues();
  // values(0) 代表主特征值，如果主特征值远大于其他两个则代表点云大致为一条直线
  HJ_INFO("[optimize] feature value: %lf", values(0) / values(1));
  return values(0) / values(1) > 100;
}

bool BuildMapWithTwoSonar::icpRegistration(int id1, int id2, SE3& icp_result) {
  mypcl::PointCloud<mypcl::PointXYZI>::Ptr target(new mypcl::PointCloud<mypcl::PointXYZI>);
  mypcl::PointCloud<mypcl::PointXYZI>::Ptr source(new mypcl::PointCloud<mypcl::PointXYZI>);
  mypcl::copyPointCloud(*(point_odom_vec_[id1].first), *target);

  auto odom_mid = point_odom_vec_[id1].second.toEigenIsometry<Eigen::Isometry3f>();
  Eigen::Matrix4d T_mid_d = odom_mid.matrix().cast<double>();
  if (id1 - 1 >= 0) {
    auto points_before = point_odom_vec_[id1 -1].first;
    auto odom_before = point_odom_vec_[id1 -1].second;
    Eigen::Isometry3f T_before = odom_before.toEigenIsometry<Eigen::Isometry3f>();
    Eigen::Matrix4d T_before_d = T_before.matrix().cast<double>();
    for (int j = 0; j < static_cast<int>(points_before->points.size()); ++j) {
      Eigen::Vector4d p(points_before->points[j].x, points_before->points[j].y, points_before->points[j].z, 1.0);
      Eigen::Vector4d points_c = T_mid_d.inverse() * (T_before_d * p);
      mypcl::PointXYZI point_base(points_c[0], points_c[1], points_c[2], 0.0);
      target->addPoint(point_base);
    }
  }

  if (id1 + 1 <= id2) {
    auto points_after= point_odom_vec_[id1 +1].first;
    auto odom_after = point_odom_vec_[id1 +1].second;
    Eigen::Isometry3f T_after = odom_after.toEigenIsometry<Eigen::Isometry3f>();
    Eigen::Matrix4d T_after_d = T_after.matrix().cast<double>();
    for (int j = 0; j < static_cast<int>(points_after->points.size()); ++j) {
      Eigen::Vector4d p(points_after->points[j].x, points_after->points[j].y, points_after->points[j].z, 1.0);
      Eigen::Vector4d points_c = T_mid_d.inverse() * (T_after_d * p);
      mypcl::PointXYZI point_base(points_c[0], points_c[1], points_c[2], 0.0);
      target->addPoint(point_base);
    }
  }

  mypcl::copyPointCloud(*(point_odom_vec_[id2].first), *source);
//  voxelGridFilter(*target, *target, 0.1);
//  voxelGridFilter(*source, *source, 0.1);

  //Ttarget_source
  Eigen::Isometry3f T_target = point_odom_vec_[id1].second.toEigenIsometry<Eigen::Isometry3f>();
  Eigen::Isometry3f T_source = point_odom_vec_[id2].second.toEigenIsometry<Eigen::Isometry3f>();
  Eigen::Isometry3f T_init = T_target.inverse() * T_source;
  Eigen::Matrix3d r = T_init.rotation().matrix().cast<double>();
  Eigen::Quaterniond q(r);
  q.normalize();
  Eigen::Vector3d t = T_init.translation().cast<double>();

  icp_result = SE3(q.toRotationMatrix(), t);

  sad::Icp3d::Options options;
  options.max_iteration_ = 50;
  options.min_effective_pts_ = 20;
  options.eps_ = 1e-4;

  sad::Icp3d icp(options);
  icp.SetSource(source);
  icp.SetTarget(target);
  mypcl::savePLYFileBinary("./target.ply", *target);
  mypcl::savePLYFileBinary("./source.ply", *source);
  bool is_icp_success = icp.AlignP2Line(icp_result);
  double icp_score = icp.GetFitnessScore(icp_result);
  HJ_INFO("[optimize] matching score: %lf", icp_score);
  if (icp_score > 0.10) {
    return false;
  }
  mypcl::PointCloud<mypcl::PointXYZI>::Ptr target_align(new mypcl::PointCloud<mypcl::PointXYZI>);  // 固定的
  mypcl::transformPointCloud(*source, *target_align, icp_result.matrix().cast<float>());
  std::cout<<"icp result = "<<icp_result.matrix().cast<float>()<<std::endl;
  mypcl::savePLYFileBinary("./align.ply", *target_align);

  return is_icp_success;
}

void BuildMapWithTwoSonar::addG2oVertex(Odom& odom) {
  Eigen::Vector3d t(odom.x, odom.y, odom.z);
  Eigen::Quaterniond rotation = Eigen::AngleAxisd(odom.yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(odom.pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(odom.roll, Eigen::Vector3d::UnitX());
  rotation.normalize();

  Eigen::Matrix3d r = rotation.toRotationMatrix();
  SE3 se3T(r, t);
  VertexPose *vp = new VertexPose();
  vp->setEstimate(se3T);
  vp->setId(vertex_id_);
  if(vertex_id_ == 0) {
    vp->setFixed(true);
  }
  optimizer_.addVertex(vp);
  vertex_id_++;
}

void BuildMapWithTwoSonar::addG2oEdge() {
  if(edge_id_ != static_cast<int>(point_odom_vec_.size()) - 2) {
    HJ_ERROR("add g2o edge error");
    return;
  }
  Odom odom1 = point_odom_vec_[edge_id_].second;
  Odom odom2 = point_odom_vec_[edge_id_ + 1].second;

  Eigen::Quaterniond rotation1 = Eigen::AngleAxisd(odom1.yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(odom1.pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(odom1.roll, Eigen::Vector3d::UnitX());
  rotation1.normalize();
  Eigen::Vector3d translation1(odom1.x, odom1.y, odom1.z);

  Eigen::Quaterniond rotation2 = Eigen::AngleAxisd(odom2.yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(odom2.pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(odom2.roll, Eigen::Vector3d::UnitX());
  rotation2.normalize();
  Eigen::Vector3d translation2(odom2.x, odom2.y, odom2.z);

  // T21
  Eigen::Matrix3d R_constraint;
  Eigen::Vector3d t_constraint;
  R_constraint = rotation2.toRotationMatrix().inverse() * rotation1.toRotationMatrix();
  t_constraint = rotation2.toRotationMatrix().inverse() * (translation1 - translation2);
  Eigen::Quaterniond q_constraint(R_constraint);
  SE3 se3_21(q_constraint, t_constraint);

  EdgeRelativeMotion* e = new EdgeRelativeMotion(); // new edge with correct cohort for caching

  VertexPose *vp0 = dynamic_cast<VertexPose *>(optimizer_.vertices().find(edge_id_)->second);
  VertexPose *vp1 = dynamic_cast<VertexPose *>(optimizer_.vertices().find(edge_id_ + 1)->second);
  e->setVertex(0, vp0); // first viewpoint
  e->setVertex(1, vp1); // second viewpoint
  e->setMeasurement(se3_21);
  // 信息矩阵，越大代表权重越大
  e->information() = 10 * Eigen::Matrix<double, 6, 6>::Identity();
  optimizer_.addEdge(e);

  edge_id_++;
}

void BuildMapWithTwoSonar::addG2oLoopEdge(int id1, int id2, SE3& icp_result) {
  SE3 se3_21 = icp_result.inverse();

  EdgeRelativeMotion *e = new EdgeRelativeMotion();
  VertexPose *vp0 = dynamic_cast<VertexPose *>(optimizer_.vertices().find(id1)->second);
  VertexPose *vp1 = dynamic_cast<VertexPose *>(optimizer_.vertices().find(id2)->second);
  vp0->setFixed(true);
  e->setVertex(0, vp0);
  e->setVertex(1, vp1);
  e->setMeasurement(se3_21);
  // 信息矩阵，越大代表权重越大
  e->information() = 1000 * Eigen::Matrix<double, 6, 6>::Identity();
  optimizer_.addEdge(e);
}

void BuildMapWithTwoSonar::optimizeFunc() {
  optimizer_.setVerbose(false);
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));
  optimizer_.setAlgorithm(solver);
  optimizer_.initializeOptimization();
  optimizer_.computeActiveErrors();  // 计算活跃误差（即当前图中所有边的误差总和）
  optimizer_.setVerbose(true);  // 设置优化器为详细输出模式
  optimizer_.optimize(50);  // 迭代25次

  for (int i = 0; i < static_cast<int>(point_odom_vec_.size()); i++) {
    VertexPose *vc = dynamic_cast<VertexPose *>(optimizer_.vertices().find(i)->second);
    SE3 pose = vc->estimate();
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = pose.unit_quaternion().toRotationMatrix();
    matrix.block<3, 1>(0, 3) = pose.translation();
    matrix_opti_vec_.push_back(matrix);
  }
}

void BuildMapWithTwoSonar::saveOneSequencePoints(const int index) {

  auto sequence_points = point_odom_vec_[index].first;
  auto odom = point_odom_vec_[index].second;
  Eigen::Isometry3f pose_f = odom.toEigenIsometry<Eigen::Isometry3f>();
  Eigen::Matrix4d pose = pose_f.matrix().cast<double>();
  for (int j = 0; j < static_cast<int>(sequence_points->points.size()); ++j) {
    Eigen::Vector4d p(sequence_points->points[j].x, sequence_points->points[j].y, sequence_points->points[j].z, 1.0);
    Eigen::Vector4d points_world = pose * p;
    mofLogoutIncc << index << " " << points_world[0] << " " << points_world[1] << " "
                  <<points_world[2] << " "<< points_world[3]<<std::endl;
  }

}


void BuildMapWithTwoSonar::savePlyFile(int &loop_id) {

  std::list<MapPointPtr> map_target_cloud_temp;
  map_target_cloud_temp.clear();

  for(int i = 0; i < static_cast<int>(point_odom_vec_.size()); i++) {
    if (i < loop_id) {
      continue;
    }
    auto p_ptr = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    Eigen::Matrix4d pose_optimized = matrix_opti_vec_[i];
    // Eigen::Isometry3f pose_before = (point_odom_vec_[i].second).toEigenIsometry<Eigen::Isometry3f>();
    for (int j = 0; j < static_cast<int>(point_odom_vec_[i].first->points.size()); ++j) {
      auto point = point_odom_vec_[i].first->points[j];
      Eigen::Vector4d p(point.x, point.y, point.z, 1);
      // Eigen::Matrix4d matrix4d = pose_before.matrix().cast<double>();
      // Eigen::Vector4d p_before = matrix4d * p;
      Eigen::Vector4d p_opt = pose_optimized * p;
      auto map_point_ptr = std::make_shared<MapPoint>(p_opt[0], p_opt[1], p_opt[2]);
      map_target_cloud_temp.push_back(map_point_ptr);
      mofLogoutIncc <<p_opt[0]<<" "<<p_opt[1]<<" "<<p_opt[2]<<" "<<p_opt[3]<<std::endl;
    }
  }

  saveMapBin(savemap_path_, map_target_cloud_temp);

  removeOutlier(map_target_cloud_temp);
  std::list<MapPointPtr> map_target_cloud_grid;
  map_target_cloud_grid = map_target_cloud_temp;
  map_target_cloud_temp.clear();
  for (auto map_ponit : map_target_cloud_grid) {
    if (!map_ponit) {  // 空指针检查
      continue;
    }
    auto gridCoord = map_ponit->hash(map_ponit, 0.05);
    if (gridMap.find(gridCoord) == gridMap.end()) {
      gridMap[gridCoord] = map_ponit;
      map_target_cloud_temp.push_back(map_ponit);
    }
  }
  std::vector<MapPointPtr> mapPointVector(map_target_cloud_temp.begin(), map_target_cloud_temp.end());
  float groundArea = calculateArea(mapPointVector);
  HJ_INFO("the pool square area = %f", groundArea);
  exportMapToAPP(map_target_cloud_temp, groundArea);
  getMapFromAPPTest();

  {
    std::unique_lock<std::mutex> lock(target_cloud_mutex_);
    map_target_cloud_.clear();
    map_target_cloud_ = map_target_cloud_temp;

  }
//  stop_buildMapping_.store(true);
  stop_sendingMsg_.store(true);
  is_building_success_.store(true);
}

float BuildMapWithTwoSonar::calculateArea(const std::vector<MapPointPtr>& sonar_pose_ptr_vec_lf) {
  std::vector<std::pair<int, float>> hight_data_;
  float width = 50.0;
  float height = 50.0;
  float resolution = 0.05;
  float width_ = static_cast<int16_t>(1.0 * width / resolution + 0.5);
  float height_ = static_cast<int16_t>(1.0 * height / resolution + 0.5);
  float resolution_ = resolution;
  float origin_x_ = -1.0 * width / 2;
  float origin_y_ = -1.0 * height / 2;
  int map_size = 1.0 * height_ * width_;
  hight_data_.resize(map_size, {0, 0.0});

  auto worldToMapEnforceBounds = [&](float wx, float wy, int& mx, int& my) {
    if (wx < origin_x_)  // wx 小于地图边界
    {
      mx = 0;
    }
    else if (wx >= resolution_ * width_ + origin_x_) // wx 大于地图边界
    {
      mx = width_ - 1;
    }
    else  // 在地图范围内
    {
      mx = (int)((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_)  // wy 小于地图边界
    {
      my = 0;
    }
    else if (wy >= resolution_ * height_ + origin_y_)  // wy 大于地图边界
    {
      my = height_ - 1;
    }
    else  // 在地图范围内
    {
      my = (int)((wy - origin_y_) / resolution_);
    }
  };

  auto raytraceLine = [&](unsigned int x0, unsigned int y0,unsigned int x1, unsigned int y1, float hight) {
      auto sign = [](int x) { return x > 0 ? 1.0 : -1.0; };

      auto bresenham2D = [&](unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                             int offset_b, unsigned int offset, unsigned int max_length, float hight) {
          unsigned int end = std::min(max_length, abs_da);
          for (unsigned int i = 0; i < end; ++i)
          {
            offset += offset_a; // 索引加1，相当于x坐标加1  // 选择(x+1, y)
            error_b += abs_db;  // 为了控制前进上升的斜率
            if ((unsigned int)error_b >= abs_da)
            {
              offset += offset_b;						// 选择(x+1, y+1)
              error_b -= abs_da;
            }
            int count = hight_data_[offset].first;
            float total_hights = hight_data_[offset].second;
            count += 1;
            total_hights += hight;
            hight_data_[offset] = {count, total_hights};
          }
      };

      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);

      // 标记x是前进一格还是后退一个单元格
      int offset_dx = sign(dx);  // ±1
      int offset_dy = sign(dy) * width_;  // 地图的宽度(栅格坐标)

      unsigned int offset = y0 * width_ + x0;  // （X0，Y0）在地图中的索引

      // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
      // 计算 scale
      double dist = hypot(dx, dy); // dist两个点之间的距离

      // if x is dominant  // x 占主导地位
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(abs_dx), hight);
        return;
      }

      // otherwise y is dominant
      int error_x = abs_dy / 2;
      bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(abs_dy), hight);
  };

  int size = sonar_pose_ptr_vec_lf.size();
  for(int i = 0; i <  size; i += 1) {
    MapPointPtr mppi = sonar_pose_ptr_vec_lf[i];
    float z_i = mppi->getCor_z();
    int map_start_x, map_start_y,map_end_x,map_end_y;
    worldToMapEnforceBounds(mppi->getCor_x(), mppi->getCor_y(), map_start_x, map_start_y);
    for(int j = i + 1; j < size; j += 1) {
      MapPointPtr mppj = sonar_pose_ptr_vec_lf[j];
      float z_j = mppj->getCor_z();
      if(mppi->distance(*mppj) < 0.05) {
        continue;
      }
      if(fabs(z_i -z_j) > 0.03) {
        continue;
      }
      float z = (z_i + z_j) / 2;
      worldToMapEnforceBounds(mppj->getCor_x(), mppj->getCor_y(), map_end_x, map_end_y);
      raytraceLine(map_start_x, map_start_y, map_end_x, map_end_y, z);
    }
  }

  int grid_number_ = 0;
  for (auto hight : hight_data_) {
    if (hight.first == 0) {  // 未更新
      continue;
    }
    grid_number_++;
  }
  float area = grid_number_ * resolution_ * resolution_;  // 面积
  return area;
}


void BuildMapWithTwoSonar::run_map() {
  int ret = pthread_setname_np(pthread_self(), "aiper_build_map");
  if (ret == 0) {
    HJ_INFO("success name aiper_build_map %s", __FUNCTION__);
  }
  else {
    HJ_INFO("failed name aiper_build_map %s", __FUNCTION__);
  }

  while (!stop_buildMapping_.load()) {
    //step 1, receive the odom pose
    {
      std::unique_lock<std::mutex> lock(get_pos_mutex_);
      receive_poses.insert(receive_poses.end(), inserted_odoms.begin(), inserted_odoms.end());
      inserted_odoms.clear();
    }

  #ifdef X9
    {
      std::unique_lock<std::mutex> lock(get_front_ultra_mutex_);
      receive_ultras.insert(receive_ultras.end(), inserted_front_ultras_.begin(), inserted_front_ultras_.end());
      inserted_front_ultras_.clear();
    }

    {
      std::unique_lock<std::mutex> lock(get_back_ultra_mutex_);
      receive_ultras.insert(receive_ultras.end(), inserted_back_ultras_.begin(), inserted_back_ultras_.end());
      inserted_back_ultras_.clear();
    }

    if (receive_poses.size() > 50) {
      receive_poses.pop_front();
    }
    if (receive_ultras.size() > 50) {
      receive_ultras.pop_front();
    }

    if (receive_poses.empty() || receive_ultras.empty()) {
      usleep(5000);
      continue;
    }
    doMappingFusion(receive_poses, receive_ultras);
    receive_poses.clear();
    receive_ultras.clear();
  #endif

  #ifdef T1_pro
    {
      std::unique_lock<std::mutex> lock(get_tof_mutex_);
      receive_tofs.insert(receive_tofs.end(), inserted_left_tofs_.begin(), inserted_left_tofs_.end());
      inserted_left_tofs_.clear();
    }
    if (receive_poses.size() > 50) {
      receive_poses.pop_front();
    }
    if (receive_tofs.size() > 50) {
      receive_tofs.pop_front();
    }
    if (receive_poses.empty() || receive_tofs.empty()) {
      usleep(5000);
      continue;
    }

    doMappingFusion(receive_poses, receive_tofs);
    receive_poses.clear();
    receive_tofs.clear();
  #endif



  }//end while
}

void BuildMapWithTwoSonar::doLoopCloseOptimization() {
  if(is_cluster_success_  && !is_optimize_success_) {
    if(point_odom_vec_.size() == 1) {
      addG2oVertex(point_odom_vec_.back().second);
      is_cluster_success_ = false;
      return;
    }
    addG2oVertex(point_odom_vec_.back().second);
    addG2oEdge();

    int loop_id = -1;
    // only test in corner
    if(!arePointsOnLine(point_odom_vec_.back().first)) {
      loop_id = loopClosureDetection();
      if (loop_id == -1 && point_odom_vec_.size() > 30) {
        HJ_ERROR("building failed, long time could not loopClosureing");
        exportMapFailInfo();
//        stop_buildMapping_.store(true);
        stop_sendingMsg_.store(true);
        is_building_success_.store(false);
        return;
      }
    }

    if(loop_id != -1 && loop_id != 0) {

      Sophus::SE3d icp_se3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

      bool is_icp_success = icpRegistration(loop_id, point_odom_vec_.size() - 1, icp_se3);
      if (!is_icp_success) {
        icp_failed_num ++;
      }
      if (icp_failed_num > 2) {
        HJ_ERROR("icp failed num is equal to %d", icp_failed_num);
        exportMapFailInfo();
//        stop_buildMapping_.store(true);
        stop_sendingMsg_.store(true);
        is_building_success_.store(false);
        return;
      }

      if(is_icp_success) {
        addG2oLoopEdge(loop_id, point_odom_vec_.size() - 1, icp_se3);
        HJ_INFO("icp_id = %d, %d",loop_id, static_cast<int>(point_odom_vec_.size()) - 1);
        optimizeFunc();
        savePlyFile(loop_id);
        int max_index = point_odom_vec_.size() -1;
        Eigen::Matrix4d T_w_old = point_odom_vec_[max_index].second.toEigenIsometry<Eigen::Isometry3f>().matrix().cast<double>();
        Eigen::Matrix4d T_w_new = matrix_opti_vec_[max_index];

        Eigen::Matrix4d T_trans = T_w_new * T_w_old.inverse();

        putIcpTransform(T_trans);

//        saveOneSequencePoints(loop_id);
//        saveOneSequencePoints(loop_id - 1);
//        saveOneSequencePoints(loop_id + 1);
//        saveOneSequencePoints(static_cast<int>(point_odom_vec_.size() - 1));
        exportMapSuccessInfo();
        is_optimize_success_ = true;
      }
    }
    is_cluster_success_ = false;
  }

}

bool BuildMapWithTwoSonar::loadMapBin() {
  std::unique_lock<std::mutex> lock(target_cloud_mutex_);
  std::ifstream input_file(loadmap_path_.c_str(), std::ios::binary);
  if (!input_file.is_open()){
    HJ_ERROR("Error opening loading map file: %s", loadmap_path_.c_str());
    return false;
  }
 
  while (true){
    float cor_x{0.0f}, cor_y{0.0f}, cor_z{0.0f};
    input_file.read(reinterpret_cast<char*>(&cor_x), sizeof(float));
    input_file.read(reinterpret_cast<char*>(&cor_y), sizeof(float));
    input_file.read(reinterpret_cast<char*>(&cor_z), sizeof(float));
    
    auto point = std::make_shared<MapPoint>(cor_x, cor_y, cor_z);
    if (input_file.eof()) {
      break;
    }
    map_target_cloud_.push_back(point);
  }
  HJ_INFO("[Mapping] Loaded map from %s success! Map size: %ld", loadmap_path_.c_str(), map_target_cloud_.size());
  input_file.close();
  return true;
}

bool BuildMapWithTwoSonar::saveMapBin(const std::string& file_path, const std::list<MapPointPtr>& point_cloud) {
  std::ofstream out_file(file_path.c_str(), std::ios::binary);
  if(!out_file.is_open()) {
    HJ_ERROR("[Mapping] Error opening saving map file!");
    return false;
  }

  for (const auto& pointPtr : point_cloud) {
    if (!pointPtr) {  // 空指针检查
      continue;
    }
    MapPoint* point = pointPtr.get();
    float cor_x = point->getCor_x();
    float cor_y = point->getCor_y();
    float cor_z = point->getCor_z();
    out_file.write(reinterpret_cast<const char*>(&cor_x), sizeof(float));
    out_file.write(reinterpret_cast<const char*>(&cor_y), sizeof(float));
    out_file.write(reinterpret_cast<const char*>(&cor_z), sizeof(float));
  }
  out_file.close();
  HJ_INFO("[Mapping] Save map point_cloud.bin success! map size is: %ld", point_cloud.size());
  return true;
}

void BuildMapWithTwoSonar::putIcpTransform(Eigen::Matrix4d &icp_transform) {
  std::unique_lock<std::mutex> lck(icp_transform_carrier_lock_);
  HJ_INFO("[Mapping] [Optimize] transform: translation: %f, %f, %f", icp_transform(0, 3), icp_transform(1, 3), icp_transform(2, 3));
  icp_transform_list.push_back(icp_transform);
}

bool BuildMapWithTwoSonar::isMappingFinished() {
  if (stop_buildMapping_.load()) {
    return true;
  } else { 
    return false;
  }
}

bool BuildMapWithTwoSonar::isStopSendingMsg() {
  if (stop_sendingMsg_.load()) {
    return true;
  } else {
    return false;
  }
}

bool BuildMapWithTwoSonar::isMapBuildingSuccess() {
  if (is_building_success_.load()) {
    return true;
  } else {
    return false;
  }
}

void BuildMapWithTwoSonar::startBuildMapping() {
  stop_buildMapping_.store(false);
}

bool BuildMapWithTwoSonar::getIcpTransform(std::vector<Eigen::Matrix4d> &icp_trans_list) {
  std::unique_lock<std::mutex> lck(icp_transform_carrier_lock_);

  if (icp_transform_list.empty()) {
    return false;
  }
  while (!icp_transform_list.empty()) {
    auto icp_trans = icp_transform_list.front();
    icp_transform_list.pop_front();
    icp_trans_list.push_back(icp_trans);
  }
  return true;
}

void BuildMapWithTwoSonar::removeOutlier(std::list<MapPointPtr> &target) {
  float query[target.size()][3];
  int i = 0;
  for (std::list<MapPointPtr>::iterator it = target.begin(); it != target.end(); ++it, ++i) {
    if (!(*it)) {  // 空指针检查
      continue;
    }
    query[i][0] = (*it)->getCor_x();
    query[i][1] = (*it)->getCor_y();
    query[i][2] = (*it)->getCor_z();
  }

  std::vector<float *> v1 = {query, query + target.size()}; // 存放query每行元素的首地址
  obstarcalib::caliblink::KDTree<float *> kdTree(v1);
  std::list<std::list<MapPointPtr>::iterator> toDelete;
  for (std::list<MapPointPtr>::iterator it = target.begin(); it != target.end(); ++it) {
    if (!(*it)) {  // 空指针检查
      continue;
    }
    float train_0 = (*it)->getCor_x();
    float train_1 = (*it)->getCor_y();
    float train_2 = (*it)->getCor_z();
    float train[3] = {train_0, train_1, train_2};
    std::vector<int> index = kdTree.knnSearch(train, 11);
    float sum_distance = 0.0;
    if (index.empty()) {
      continue;
    }
    for (std::size_t j = 1; j < index.size(); ++j) {
      sum_distance += std::sqrt((query[index[j]][0] - train_0) * (query[index[j]][0] - train_0) +
              (query[index[j]][1] - train_1) * (query[index[j]][1] - train_1) +
              (query[index[j]][2] - train_2) * (query[index[j]][2] - train_2));
    }
    if (sum_distance / (index.size() - 1) > 0.10) {
      toDelete.push_back(it);
    }
  }
  for (auto it : toDelete) {
    target.erase(it);
  }

}


#ifdef X9
void BuildMapWithTwoSonar::doMappingFusion(std::deque<Odom> &receive_poses, std::deque<ReceivedSonar> &receive_ultras) {
  if (!set_first_odom) {
    current_odom = receive_poses.front();
    init_timeStamp = current_odom.timestmap;
    set_first_odom = true;
    receive_poses.clear();
    receive_ultras.clear();
    return;
  }

  for (auto pos_iter = receive_poses.begin(); pos_iter != receive_poses.end(); pos_iter++) {
    uint64_t pos_timestamp = pos_iter->timestmap;
    current_odom = *pos_iter;
    double time_interval = static_cast<double>(current_odom.timestmap - init_timeStamp) * 1e-6;
    if (static_cast<int>(time_interval) % 100 == 0) {
      HJ_INFO("map_time_diff = %lf", time_interval);
    }
    if (time_interval > 450.0) {
      exportMapFailInfo();
//      stop_buildMapping_.store(true);
      stop_sendingMsg_.store(true);
      is_building_success_.store(false);
      return;
    }
    for (auto ultra_iter = receive_ultras.begin(); ultra_iter != receive_ultras.end(); ultra_iter++) {
      if (ultra_iter->status == SonarStatus::back) {
        continue;
      }
      uint64_t ultra_timestamp = ultra_iter->timestamp;
      float left_front_length = ultra_iter->distance;
      if (left_front_length - (-1.0) < 1e-3) {
        continue;
      }
//      HJ_INFO("[mapping] pos_time = %ld, tof_time = %ld, diff_time = %ld", pos_timestamp, ultra_timestamp, pos_timestamp - ultra_timestamp);
      uint64_t result_diff_time = (ultra_timestamp > pos_timestamp) ? (ultra_timestamp - pos_timestamp) : (
              pos_timestamp - ultra_timestamp);
      if (result_diff_time > 20000) { // 15ms
        continue;
      }
      MapPointPtr sonar_world_pose_ptr;
      SynchronizedData odom_and_sonar_world;
      getSonarPoint(left_front_length, ultra_iter->status, current_odom, sonar_world_pose_ptr);
      {
        std::unique_lock<std::mutex> lock(target_cloud_mutex_);
        map_target_cloud_.push_back(sonar_world_pose_ptr);
        if (static_cast<int>(map_target_cloud_.size()) % 100 == 0) {
          HJ_INFO("map_target_cloud_ size = %ld", map_target_cloud_.size());
        }
      }

      odom_and_sonar_world.map_point = *sonar_world_pose_ptr;
      odom_and_sonar_world.odom = current_odom;
      cluster(odom_and_sonar_world);//odom time pose
      doLoopCloseOptimization();

    }
  }

}
#endif

#ifdef T1_pro
void BuildMapWithTwoSonar::doMappingFusion(std::deque<Odom> &receive_poses, std::deque<ReceivedTof> &receive_tofs) {
  if (!set_first_odom && !receive_poses.empty()) {
    current_odom = receive_poses.front();
    init_timeStamp = current_odom.timestmap;
    set_first_odom = true;
    return;
  }

  for (auto pos_iter = receive_poses.begin(); pos_iter != receive_poses.end(); pos_iter++) {
    uint64_t pos_timestamp = pos_iter->timestmap;
    current_odom = *pos_iter;
    double time_interval = static_cast<double>(current_odom.timestmap - init_timeStamp) * 1e-6;
    if (static_cast<int>(time_interval) % 100 == 0) {
      HJ_INFO("map_time_diff = %lf", time_interval);
    }
    if (time_interval > 450.0) {
      exportMapFailInfo();
//      stop_buildMapping_.store(true);
      stop_sendingMsg_.store(true);
      is_building_success_.store(false);
      return;
    }
    for (auto tof_iter = receive_tofs.begin(); tof_iter != receive_tofs.end(); tof_iter++) {
      uint64_t tof_timestamp = tof_iter->timestamp;
      float left_front_length = tof_iter->front_distance;
      float left_back_length = tof_iter->back_distance;
      if (left_front_length - (-1.0) < 1e-3) {
        continue;
      }
//      HJ_INFO("[mapping] pos_time = %ld, tof_time = %ld, diff_time = %ld", pos_timestamp, tof_timestamp, pos_timestamp - tof_timestamp);
      uint64_t result_diff_time = (tof_timestamp > pos_timestamp) ? (tof_timestamp - pos_timestamp) : (
              pos_timestamp - tof_timestamp);
      if (result_diff_time > 20000) { // 15ms
        continue;
      }

      MapPointPtr sonar_world_pose_ptr;
      SynchronizedData odom_and_sonar_world;
      getSonarPoint(left_front_length, SonarStatus::front, current_odom, sonar_world_pose_ptr);
      {
        std::unique_lock<std::mutex> lock(target_cloud_mutex_);
        map_target_cloud_.push_back(sonar_world_pose_ptr);
        if (static_cast<int>(map_target_cloud_.size()) % 100 == 0) {
          HJ_INFO("map_target_cloud_ size = %ld", map_target_cloud_.size());
        }
      }

      odom_and_sonar_world.map_point = *sonar_world_pose_ptr;
      odom_and_sonar_world.odom = current_odom;
      cluster(odom_and_sonar_world);//odom time pose
      doLoopCloseOptimization();
    }
  }

}
#endif

} // end namespace HJ_mapping
} // end namespace HJ_slam