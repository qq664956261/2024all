#include "map_modules/icp_match.h"

static std::mutex icp_transform_carrier_lock_;
static std::deque<Eigen::Matrix4d> icp_transform_list;

// TODO(edan): zhen.liang 静态全局变量改成属性？
static bool bstart_icp_flag = false;
static bool bis_once_icp_finish = true; 

static Eigen::Matrix4f last_icp_trans;
static uint64_t last_icp_timestamp;


uint64_t getCurrentTimestampUs() {
  //us time
  return static_cast<uint64_t>(ros::Time::now().toNSec()*0.001);
}

IcpMatcher::IcpMatcher(const rapidjson::Value &json_conf) {
  icp_config_parameters_.loadParameters(json_conf);
  // if (icp_config_parameters_.load_map_as_target) {
  //   loadMapBin();
  // }
  icp_ofs_log_.open(icp_config_parameters_.savelog_path + "/icplog.txt");

#ifdef map_debug
  target_ofs_log_.open(icp_config_parameters_.savelog_path_ + "/target_log.txt");
  source_ofs_log_.open(icp_config_parameters_.savelog_path_ + "/source_log.txt");
#endif

  gthreadIcpMatchptr_ = new std::thread(&IcpMatcher::run_icp, this);
  last_icp_trans.setIdentity();
  last_icp_timestamp = getCurrentTimestampUs(); // us
}

void IcpMatcher::reset() {
  icp_ofs_log_ << __FUNCTION__ << " " << ros::Time::now() << std::endl;

  source_cloud_.clear();
  target_cloud_.clear();
  icp_count_ = 0;
  stop_icp_ = false;
  sourceNotIncrease = false;

}

IcpMatcher::~IcpMatcher() {
  source_cloud_.clear();
  target_cloud_.clear();
  icp_ofs_log_.close();
#ifdef map_debug
  target_ofs_log_.close();
  source_ofs_log_.close();
#endif
  if(gthreadIcpMatchptr_ != nullptr) {
    delete gthreadIcpMatchptr_;
    gthreadIcpMatchptr_ = nullptr;
  }
}

bool IcpMatcher::getIcpStopFlag() {
  return stop_icp_;
}

void IcpMatcher::setIcpStopFlag(const bool icp_stop) {
  stop_icp_ = true;
}

#ifdef optmization_g20
Eigen::Matrix4d IcpMatcher::doIcpMatchingByOptmization() {
  std::vector<IcpPoint> pts_target, pts_source;
  pts_target.clear();
  pts_source.clear();
  for (auto it_target = target_cloud_.begin(); it_target != target_cloud_.end(); ++it_target) {
    float x_ = (*it_target)->getCor_x();
    float y_ = (*it_target)->getCor_y();
    float z_ = (*it_target)->getCor_z();
    pts_target.push_back(IcpPoint(x_, y_, z_));
  }

  for (auto it_source = source_cloud_.begin(); it_source != source_cloud_.end(); ++it_source) {
    float x_ = (*it_source)->getCor_x();
    float y_ = (*it_source)->getCor_y();
    float z_ = (*it_source)->getCor_z();
    pts_source.push_back(IcpPoint(x_, y_, z_));
  }

  std::vector<IcpPoint> pts_target_simple;
  NEIGHBOR neighbor_ = findNearest(pts_source, pts_target);
  for(int j = 0; j < pts_source.size(); j++) {
    pts_target_simple.push_back(IcpPoint(pts_target[neighbor_.indices[j]].x, pts_target[neighbor_.indices[j]].y, pts_target[neighbor_.indices[j]].z));
  }

  Eigen::Matrix4d result_estimate = getIcpBA(pts_target_simple, pts_source);
  return result_estimate;

}
#endif

ICP_OUT IcpMatcher::doIcpMatching(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, double tolerance) {
  int row_src = A.rows();
  int row_dst = B.rows();
  Eigen::MatrixXd src = Eigen::MatrixXd::Ones(3+1, row_src);//4*row_src
  Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(3, row_src);//3*row_src
  Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(3+1, row_dst);//4*row_dst
  NEIGHBOR neighbor;
  Eigen::Matrix4d T;
  Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(3,row_src);//3*30
  ICP_OUT result;
  int iter = 0;

  for (int i = 0; i<row_src; i++) {
    src.block<3, 1>(0, i) = A.block<1,3>(i,0).transpose();
    src3d.block<3, 1>(0, i) = A.block<1,3>(i,0).transpose();
  }

  for (int i = 0; i < row_dst; i++) {
    dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
  }

  double prev_error = 0;
  double mean_error = 0;

  for (int i = 0; i < max_iterations; i++) {
    neighbor = findNearestValue(src3d.transpose(), B);

    for(int j = 0; j < row_src; j++) {
      dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
    }

    T = GetTransformMatrix(src3d.transpose(), dst_chorder.transpose());

    src = T * src;
    for(int j = 0; j < row_src; j++) {
      src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
    }

    mean_error = std::accumulate(neighbor.distances.begin(), neighbor.distances.end(),0.0) / neighbor.distances.size();
    icp_ofs_log_ << "mean_error: " << mean_error << "iter: "<<i<< std::endl;

    if (fabs(prev_error - mean_error) < tolerance) {
      iter = i;
      break;
    }
    prev_error = mean_error;
    iter = i;
  }

  T = GetTransformMatrix(A, src3d.transpose());

  result.trans = T;
  result.distances = neighbor.distances;
  result.iter = iter;

  return result;
}

std::list<MapPointPtr>& IcpMatcher::getSourceCloud(){
  return source_cloud_;
}

std::list<MapPointPtr>& IcpMatcher::getTargetCloud(){
  return target_cloud_;
}

NEIGHBOR IcpMatcher::findNearestValue(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst) {
  NEIGHBOR neigh;
  float query[dst.rows()][3];
  for (int i = 0; i < dst.rows(); ++i) {
    query[i][0] = static_cast<float>(dst(i, 0));
    query[i][1] = static_cast<float>(dst(i, 1));
    query[i][2] = static_cast<float>(dst(i, 2));
  }

  std::vector<float *> v1 = {query, query + dst.rows()}; // 存放query每行元素的首地址
  obstarcalib::caliblink::KDTree<float *> kdTree(v1);


  for (int j = 0; j < src.rows(); ++j) {
    float train_0 = static_cast<float>(src(j, 0));
    float train_1 = static_cast<float>(src(j, 1));
    float train_2 = static_cast<float>(src(j, 2));
    float train[3] = {train_0, train_1, train_2};
    std::vector<int> index = kdTree.knnSearch(train, 1);
    float min_diatance = sqrt((query[index[0]][0] - train_0) * (query[index[0]][0] - train_0)
      + (query[index[0]][1] - train_1) * (query[index[0]][1] - train_1)
      + (query[index[0]][2] - train_2) * (query[index[0]][2] - train_2));


    neigh.distances.push_back(min_diatance);
    neigh.indices.push_back(index[0]);

  }

  return neigh;
}

NEIGHBOR IcpMatcher::findNearest(const std::vector<IcpPoint> &source, const std::vector<IcpPoint> &target) {
  NEIGHBOR neigh;

  float query[target.size()][3];
  for (int i = 0; i < static_cast<int>(target.size()); ++i) {
    query[i][0] = target[i].x;
    query[i][1] = target[i].y;
    query[i][2] = target[i].z;
  }

  std::vector<float *> v1 = {query, query + target.size()}; // 存放query每行元素的首地址
  obstarcalib::caliblink::KDTree<float *> kdTree(v1);


  for (int j = 0; j < static_cast<int>(source.size()); ++j) {
    float train_0 = source[j].x;
    float train_1 = source[j].y;
    float train_2 = source[j].z;
    float train[3] = {train_0, train_1, train_2};
    std::vector<int> index = kdTree.knnSearch(train, 1);
    float min_diatance = sqrt((query[index[0]][0] - train_0) * (query[index[0]][0] - train_0)
                              + (query[index[0]][1] - train_1) * (query[index[0]][1] - train_1)
                              + (query[index[0]][2] - train_2) * (query[index[0]][2] - train_2));

    neigh.distances.push_back(min_diatance);
    neigh.indices.push_back(index[0]);

  }

  return neigh;
}


Eigen::Matrix4d IcpMatcher::GetTransformMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
  /*
  Notice:
  1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
  2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
  */
  Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
  Eigen::Vector3d centroid_A(0, 0, 0);
  Eigen::Vector3d centroid_B(0, 0, 0);
  Eigen::MatrixXd AA = A;
  Eigen::MatrixXd BB = B;
  int row = A.rows();

  for(int i = 0; i < row; i++){
    centroid_A += A.block<1,3>(i,0).transpose();
    centroid_B += B.block<1,3>(i,0).transpose();
  }
  centroid_A /= row;
  centroid_B /= row;
  for(int i = 0; i < row; i++){
    AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
    BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
  }

  Eigen::MatrixXd H = AA.transpose() * BB;
  Eigen::MatrixXd U;
  Eigen::VectorXd S;
  Eigen::MatrixXd V;
  Eigen::MatrixXd Vt;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  S = svd.singularValues();
  V = svd.matrixV();
  Vt = V.transpose();

  R = Vt.transpose()*U.transpose();

  if (R.determinant() < 0 ){
    Vt.block<1, 3>(2,0) *= -1;
    R = Vt.transpose() * U.transpose();
  }

  t = centroid_B - R * centroid_A;

  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;

}

Eigen::MatrixXd IcpMatcher::prepareDataForICP(std::list<MapPointPtr> &data_cloud) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(data_cloud.size(), 3);

  int i = 0;
  for (auto it_cloud = data_cloud.begin(); it_cloud != data_cloud.end(); ++it_cloud, ++i) {
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(1, 3);
    tmp << (*it_cloud)->getCor_x(), (*it_cloud)->getCor_y(), (*it_cloud)->getCor_z();
    A.block<1 , 3>(i, 0) = tmp;
  }
  return A;

}


bool IcpMatcher::setInputSource(std::list<MapPointPtr> &source_cloud, uint64_t cloud_timestamp) {
  if(!bis_once_icp_finish) {
    return false;
  }
  if (sourceNotIncrease) {
    return false;
  }
  source_cloud_.insert(source_cloud_.end(), source_cloud.begin(), source_cloud.end());
  //todo if cloud is less
  if (static_cast<int>(source_cloud_.size()) >= source_buff_size_ || stop_icp_) {

//    icp_time_list_.push_back(cloud_timestamp);
    //cloud_timestamp_ = cloud_timestamp;
    bstart_icp_flag = true;
  }
  if (stop_icp_) {
    sourceNotIncrease = true;
  }
  return true;

}

bool IcpMatcher::setInputTarget(std::list<MapPointPtr> &target_cloud, uint64_t cloud_timestamp) {
  if(!bis_once_icp_finish) {
    return false;
  }
  if(target_cloud.empty()) {
    return false;
  }

  target_cloud_.insert(target_cloud_.end(), target_cloud.begin(), target_cloud.end());

  return true;
}

void IcpMatcher::setSourceBufferSize(const int size) {
  source_buff_size_ = size;
}

int IcpMatcher::getTargetCloudSize() {
  return target_cloud_.size();
}

int IcpMatcher::getSourceCloudSize() {
  return source_cloud_.size();
}

void IcpMatcher::errorPropagation(std::list<MapPointPtr> &point_cloud, const Eigen::Matrix4f &transform, int start_idx, int end_idx, bool is_const_trans) {
  Eigen::Matrix3f r_matrix = Eigen::Matrix3f::Identity();
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      r_matrix(i, j) = transform(i, j);
    }
  }
  Eigen::Vector3f t_vector(transform(0, 3), transform(1, 3), transform(2, 3));
  Eigen::Vector3f zero_vector(0.f, 0.f, 0.f);
  Eigen::Quaternionf q = Eigen::Quaternionf(r_matrix);
  Eigen::Quaternionf q_pi(0.f, 0.f, 1.f, 0.f);

  icp_ofs_log_ << "init angle: " << q_pi.angularDistance(q) * 180 / 3.1415926  << std::endl;
  icp_ofs_log_ << "init t_vector: " << t_vector.transpose()  << std::endl;
  icp_ofs_log_ << "q: " << q.toRotationMatrix().eulerAngles(0,1,2).transpose()[2] * 180 / 3.1415926 << std::endl;

  for (auto it = point_cloud.begin(); it != point_cloud.end(); ++it) {
    float weight_t_noConst = static_cast<float>(start_idx) / end_idx;
    float weight_t = is_const_trans ? 1.0 : weight_t_noConst;
    Eigen::Vector3f point((*it)->getCor_x(), (*it)->getCor_y(), (*it)->getCor_z());
    Eigen::Vector3f angle_vector = q.toRotationMatrix().eulerAngles(0,1,2);
    Eigen::Vector3f interpolation_angle_vector = zero_vector + (angle_vector - zero_vector) * weight_t;
    Eigen::Matrix3f interpolation_rotationMatrix = Eigen::AngleAxisf(interpolation_angle_vector.norm(), interpolation_angle_vector.normalized()).toRotationMatrix();
    Eigen::Quaternionf interpolation_q(interpolation_rotationMatrix);

    Eigen::Vector3f interpolation_t_vector = zero_vector + (t_vector - zero_vector) * weight_t;
//    Eigen::Vector3f point_transformed = interpolation_t_vector + interpolation_q * point;
    Eigen::Vector3f point_transformed = interpolation_t_vector + point;

    (*it)->alterCor_x(point_transformed[0]);
    (*it)->alterCor_y(point_transformed[1]);
    (*it)->alterCor_z(point_transformed[2]);
    start_idx++;

  }

}


void IcpMatcher::extrapolate_error(const Eigen::Matrix4f& trans_input, const float interplolation_coeff, Eigen::Matrix4f& trans_res) {
  Eigen::Matrix3f matrix = trans_input.block<3, 3>(0, 0);
  Eigen::Vector3f angle = matrix.eulerAngles(0, 1, 2);
  Eigen::Vector3f zero_angle = Eigen::Vector3f(0.f, 0.f, 0.f);
  Eigen::Vector3f interplolation_angle = zero_angle + (angle - zero_angle) * interplolation_coeff;
  Eigen::Matrix3f interplolation_matrix = Eigen::AngleAxisf(interplolation_angle.norm(), interplolation_angle.normalized()).toRotationMatrix();

  Eigen::Vector3f zero_translation = Eigen::Vector3f(0.f, 0.f, 0.f);
  Eigen::Vector3f translation = trans_input.block<3, 1>(0, 3);
  Eigen::Vector3f interplolation_translation = zero_translation + (translation - zero_translation) * interplolation_coeff;

  trans_res.block<3, 3>(0, 0) = interplolation_matrix;
  trans_res.block<3, 1>(0, 3) = interplolation_translation;
}

void IcpMatcher::correctTargetCloud(Eigen::Matrix4d &trans_input) {
  icp_ofs_log_ << "****************  correct target cloud  ****************" << std::endl;
  icp_ofs_log_ << trans_input << std::endl;
  icp_ofs_log_ << "****************correct target cloud end****************" << std::endl;

#ifdef map_debug
  for (auto it_cloud = target_cloud_.begin(); it_cloud != target_cloud_.end(); ++it_cloud) {
    target_ofs_log_ << (*it_cloud)->getCor_x() <<" " <<(*it_cloud)->getCor_y() << " " << (*it_cloud)->getCor_z() << std::endl;
  }
  target_ofs_log_ << "****************correct target cloud end****************" << std::endl;
#endif
  Eigen::Matrix4f trans_input_ = trans_input.cast<float>();
  errorPropagation(target_cloud_, trans_input_, 0, target_cloud_.size(), false); // 校正地图

#ifdef map_debug
  for (auto it_cloud = target_cloud_.begin(); it_cloud != target_cloud_.end(); ++it_cloud) {
    target_ofs_log_ << (*it_cloud)->getCor_x() <<" " <<(*it_cloud)->getCor_y() << " " << (*it_cloud)->getCor_z() << std::endl;
  }
#endif
}

void IcpMatcher::correctSourceCloud() {
#ifdef map_debug
  for (auto it_cloud = source_cloud_.begin(); it_cloud != source_cloud_.end(); ++it_cloud) {
    source_ofs_log_ << (*it_cloud)->getCor_x() <<" " <<(*it_cloud)->getCor_y() << " " << (*it_cloud)->getCor_z() << std::endl;
  }
  source_ofs_log_ << "****************correct source cloud end****************" << std::endl;
#endif

//  if(!is_icp_first_corrected_)
//    errorPropagation(source_cloud_, icp_transform_, target_cloud_.size(), target_cloud_.size(), true);
//  else
//    errorPropagation(source_cloud_, icp_transform_, 0, source_cloud_.size(), true);

#ifdef map_debug
  for (auto it_cloud = source_cloud_.begin(); it_cloud != source_cloud_.end(); ++it_cloud) {
    source_ofs_log_ << (*it_cloud)->getCor_x() <<" " <<(*it_cloud)->getCor_y() << " " << (*it_cloud)->getCor_z() << std::endl;
  }
#endif

}

void IcpMatcher::correctSourceCloud(Eigen::Matrix4d &icp_trans) {
  Eigen::Matrix4d icp_transform_d = icp_trans.cast<double>();
  auto rotation = icp_transform_d.topLeftCorner<3, 3>();
  auto translation = icp_transform_d.topRightCorner<3, 1>();

  for (auto it_cloud = source_cloud_.begin(); it_cloud != source_cloud_.end(); ++it_cloud) {
    Eigen::MatrixXd src_point(3, 1);
    src_point<<(*it_cloud)->getCor_x(), (*it_cloud)->getCor_y(), (*it_cloud)->getCor_z();
    Eigen::MatrixXd dst(3, 1);
    dst = rotation * src_point + translation;
    (*it_cloud)->alterCor_x(dst(0));
    (*it_cloud)->alterCor_y(dst(1));
    (*it_cloud)->alterCor_z(dst(2));
  }

}

bool IcpMatcher::saveMapBin(const std::string& file_path, const std::list<MapPointPtr>& point_cloud) {
  std::ofstream out_file(file_path.c_str(), std::ios::binary);
  if(!out_file.is_open()) {
    HJ_ERROR("Error opening saving map file!");
    return false;
  }

  for (const auto& pointPtr : point_cloud) {
    MapPoint* point = pointPtr.get();
    float cor_x = point->getCor_x();
    float cor_y = point->getCor_y();
    float cor_z = point->getCor_z();
    out_file.write(reinterpret_cast<const char*>(&cor_x), sizeof(float));
    out_file.write(reinterpret_cast<const char*>(&cor_y), sizeof(float));
    out_file.write(reinterpret_cast<const char*>(&cor_z), sizeof(float));
  }
  out_file.close();
  HJ_INFO("Save map point_cloud.bin success! map size is: %ld", point_cloud.size());
  return true;
}

bool IcpMatcher::loadMapBin(const std::string& file_path) {
  std::ifstream input_file(file_path.c_str(), std::ios::binary);
  if (!input_file.is_open()){
    HJ_ERROR("Error opening loading map file: %s", file_path.c_str());
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
    target_cloud_.push_back(point);
  }
  HJ_INFO("Loaded map from map/point_cloud.bin success! Map size: %ld", target_cloud_.size());
  input_file.close();
  return true;
}

void IcpMatcher::reduceSourceCloud() {
  for (int j = 0; j < source_buff_size_ / 4; ++j) {
    source_cloud_.pop_front();
  }
  bstart_icp_flag = false;
}

//void IcpMatcher::correctIcpTransform() {
//  Eigen::Matrix4f source_cloud_transform;
//  float interplolation_coeff = static_cast<float>(source_cloud_.size()) / target_cloud_.size();
//  extrapolate_error(icp_transform_, interplolation_coeff, source_cloud_transform);
//  icp_transform_ =  source_cloud_transform * icp_transform_;
//}

void IcpMatcher::putIcpTransform(Eigen::Matrix4d &icp_transform) {
  std::unique_lock<std::mutex> lck(icp_transform_carrier_lock_);
  HJ_INFO("icp_transform: translation: %f, %f, %f", icp_transform(0, 3), icp_transform(1, 3), icp_transform(2, 3));
  icp_transform_list.push_back(icp_transform);
}

bool IcpMatcher::getIcpTransform(std::vector<Eigen::Matrix4d> &icp_trans_list) {
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


#ifdef optmization_g20
Eigen::Matrix4d IcpMatcher::getIcpBA (std::vector<IcpPoint> &pts_target, std::vector<IcpPoint> &pts_source) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

  result = bundleAdjustment(pts_target, pts_source);
  return result;

}
#endif

#ifdef optmization_g20
Eigen::Matrix4d IcpMatcher::bundleAdjustment (
        const std::vector<IcpPoint>& pts1,
        const std::vector<IcpPoint>& pts2) {

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
  std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverEigen<Block::PoseMatrixType>());
  std::unique_ptr<Block> solver_ptr (new Block (std::move(linearSolver)));
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // vertex
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(
          Eigen::Matrix3d::Identity(),
          Eigen::Vector3d(0,0,0 )));
  optimizer.addVertex(pose);

  // edges
  int index = 1;
  std::vector<EdgeProjectXYZRGBDPoseOnly*> edges;
  for (size_t i = 0; i < pts1.size(); i++) {
    EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
    edge->setId(index);
    edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*> (pose));
    edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity());
    g2o::RobustKernelHuber* robust_kernel_huber = new g2o::RobustKernelHuber;
    robust_kernel_huber->setDelta(0.2); //设置delta大小
    edge->setRobustKernel(robust_kernel_huber); //向边添加鲁棒核函数
    optimizer.addEdge(edge);
    index++;
    edges.push_back(edge);
  }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.setVerbose( true );
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
  std::cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<std::endl;

  icp_ofs_log_ << "icp BA T = " << Eigen::Isometry3d(pose->estimate()).matrix() << std::endl;

  icp_ofs_log_ << "icp BA chi2 = " << optimizer.chi2()<<std::endl;
  auto optimize_result = Eigen::Isometry3d(pose->estimate()).matrix();
  Eigen::Matrix4d icp_result = Eigen::Matrix4d::Identity();
  icp_result << static_cast<double>(optimize_result(0, 0)), static_cast<double>(optimize_result(0, 1)), static_cast<double>(optimize_result(0, 2)), static_cast<double>(optimize_result(0, 3)),
          static_cast<double>(optimize_result(1, 0)), static_cast<double>(optimize_result(1, 1)), static_cast<double>(optimize_result(1, 2)), static_cast<double>(optimize_result(1, 3)),
          static_cast<double>(optimize_result(2, 0)), static_cast<double>(optimize_result(2, 1)), static_cast<double>(optimize_result(2, 2)), static_cast<double>(optimize_result(2, 3)),
          static_cast<double>(optimize_result(3, 0)), static_cast<double>(optimize_result(3, 1)), static_cast<double>(optimize_result(3, 2)), static_cast<double>(optimize_result(3, 3));
  return icp_result;
}
#endif


void IcpMatcher::run_icp() {
  int ret = pthread_setname_np(pthread_self(), "aiper_icp");
  if(ret == 0) {
    HJ_INFO("success name aiper_icp: %s", __FUNCTION__);
  }
  else {
    HJ_INFO("failed name aiper_icp: %s", __FUNCTION__);
  }

  while(true) {

    // TODO(edan): 阻塞等待？
    if(!bstart_icp_flag) {
        usleep(5000);
        continue;
    }

    if (!stop_icp_) {
      icp_ofs_log_ << "********************* icp corrected *********************" << std::endl;
      icp_count_++;
      Eigen::MatrixXd src_cloud = prepareDataForICP(source_cloud_);
      Eigen::MatrixXd dst_cloud = prepareDataForICP(target_cloud_);
      Eigen::Matrix4d icp_transform = Eigen::Matrix4d::Identity();
  #ifdef optmization_g20
      icp_transform = doIcpMatchingByOptmization();
  #else
      ICP_OUT result = doIcpMatching(src_cloud, dst_cloud);
      icp_transform = result.trans;
  #endif

      icp_ofs_log_ << "icp count = " << icp_count_ << std::endl;
      icp_ofs_log_ << "icp result source size = " << source_cloud_.size() << std::endl;
      icp_ofs_log_ << "icp result target = " << target_cloud_.size() << std::endl;
      icp_ofs_log_ << "icp result by svd = " << result.trans << std::endl;
      icp_ofs_log_ << "icp result by optimized = " << icp_transform << std::endl;
      icp_ofs_log_ << "********************* corrected *********************" << std::endl;
      // if (icp_count_ == 1) {
      //   //      correctTargetCloud();
      //   saveMapBin();
      // }
      //todo
      correctSourceCloud(icp_transform);
      //correctIcpTransform();
      reduceSourceCloud();
      putIcpTransform(icp_transform);

    }
//    else {
//      icp_ofs_log_ << "stop icp source cloud size = "<<source_cloud_.size()<<std::endl;
//
//      icp_ofs_log_ << "********************* icp corrected *********************" << std::endl;
//      icp_count_++;
//      Eigen::MatrixXd src_cloud = prepareDataForICP(source_cloud_);
//      Eigen::MatrixXd dst_cloud = prepareDataForICP(target_cloud_);
//      ICP_OUT result = doIcpMatching(src_cloud, dst_cloud);
//      icp_transform_ = result.trans;
//      icp_ofs_log_ << "icp count = " << icp_count_<<std::endl;
//      icp_ofs_log_ << "icp result source size = " << source_cloud_.size()<<std::endl;
//      icp_ofs_log_ << "icp result target = " << target_cloud_.size()<<std::endl;
//      icp_ofs_log_ << "icp result = " << icp_transform_<<std::endl;
//      icp_ofs_log_ << "********************* corrected *********************" << std::endl;
//      putIcpTransform(icp_transform_);
//      bstart_icp_flag = false;
//
//    }


  }
    icp_ofs_log_ << __FUNCTION__ << " end while" << std::endl;
}
