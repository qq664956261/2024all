#ifndef ICP_MATCH_H
#define ICP_MATCH_H
#include "map_modules/KDTree.h"
#include "map_modules/mapPoint.h"
#ifdef optmization_g20
#include "map_modules/icp_estimate.h"
#endif
#include "log.h"
#include "rapidjson/document.h"
#include "config_parameters.h"

#include <iostream>
#include <fstream>
#include <list>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <ros/ros.h>

//#define map_debug


class MapPoint;
typedef std::shared_ptr<MapPoint> MapPointPtr;

struct IcpPoint {
  float x;
  float y;
  float z;
  IcpPoint();
  IcpPoint(float x_, float y_, float z_)
          : x(x_), y(y_), z(z_) { };

};
typedef struct {
  Eigen::Matrix4d trans;
  std::vector<float> distances;
  int iter;
} ICP_OUT;


typedef struct {
  std::vector<float> distances;
  std::vector<int> indices;
} NEIGHBOR;

class IcpMatcher {
 public:

  explicit IcpMatcher(const rapidjson::Value &json_conf);
  ~IcpMatcher();

  void reset();

  bool setInputSource(std::list<MapPointPtr> &source_cloud, uint64_t cloud_timestamp);

  bool setInputTarget(std::list<MapPointPtr> &target_cloud, uint64_t cloud_timestamp);

  void setSourceBufferSize(const int size);

  int getTargetCloudSize();

  int getSourceCloudSize();

  void reduceSourceCloud();

  Eigen::Matrix4f getIcpTrans();

  bool getIcpTransform(std::vector<Eigen::Matrix4d> &icp_trans_list);

  bool getIcpStopFlag();

  void setIcpStopFlag(const bool icp_stop);

  ICP_OUT doIcpMatching(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations = 100, double tolerance = 1e-6);
  std::list<MapPointPtr>& getSourceCloud();
  std::list<MapPointPtr>& getTargetCloud();
  bool saveMapBin(const std::string& file_path, const std::list<MapPointPtr>& point_cloud);
  bool loadMapBin(const std::string& file_path);

 private:

  void run_icp();
  //map
  void correctTargetCloud(Eigen::Matrix4d &trans_input);
  void correctSourceCloud();

  void correctSourceCloud(Eigen::Matrix4d &icp_trans);

  void extrapolate_error(const Eigen::Matrix4f& trans_input, const float interplolation_coeff, Eigen::Matrix4f& trans_res);

  void errorPropagation(std::list<MapPointPtr> &point_cloud, const Eigen::Matrix4f &transform, int start_idx, int end_idx, bool is_const);
  void correctIcpTransform();
  void putIcpTransform(Eigen::Matrix4d &icp_transform);
  NEIGHBOR findNearestValue(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
  NEIGHBOR findNearest(const std::vector<IcpPoint> &source, const std::vector<IcpPoint> &target);
  Eigen::Matrix4d GetTransformMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
  Eigen::MatrixXd prepareDataForICP(std::list<MapPointPtr> &data_cloud);
  Eigen::Matrix4d getIcpBA (std::vector<IcpPoint> &pts_target, std::vector<IcpPoint> &pts_source);
#ifdef optmization_g20
  Eigen::Matrix4d doIcpMatchingByOptmization();
#endif
  Eigen::Matrix4d bundleAdjustment(const std::vector<IcpPoint>& pts1, const std::vector<IcpPoint>& pts2);
  std::list<MapPointPtr> source_cloud_;
  std::list<MapPointPtr> target_cloud_;
  int source_buff_size_ = 0;

  std::thread* gthreadIcpMatchptr_ = nullptr;

  std::ofstream icp_ofs_log_;
  ICPConfigParameters icp_config_parameters_;
#ifdef map_debug
  std::ofstream target_ofs_log_;
  std::ofstream source_ofs_log_;
#endif

  int icp_count_ = 0;
  // TODO(edan): How does the relocalization control this.
  bool stop_icp_ = false;
  bool sourceNotIncrease = false;

};
#endif // ICP_MATCH_H
