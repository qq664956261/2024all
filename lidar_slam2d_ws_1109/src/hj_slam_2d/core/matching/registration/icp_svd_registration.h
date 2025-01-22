/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_

#include "registration_interface.h"
#include "../../../util/knn/KDTreeTableAdaptor.h"
#include "../../../util/point_cloud.h"
#include "../../../util/grid_map/probability_grid.h"
#include "../../../util/grid_map/grid_2d.h"

#include "../real_time_correlative_scan_matcher_2d.h"
#include <memory>
// #include <unique_ptr.h>
// #include <pcl/kdtree/kdtree_flann.h>

// create the kdtree
typedef KDTreeTableAdaptor<float, float> KDTree;

namespace hjSlam_2d {

class ICPSVDRegistration: public RegistrationInterface {
  public:
    ICPSVDRegistration();
    ICPSVDRegistration(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    bool setInputTarget(const hjSlam_2d::PointCloud& input_target) override;
    bool scanMatch(
        const hjSlam_2d::PointCloud &input_source) override;

    bool scanMatch(const hjSlam_2d::PointCloud &input_source, const mapping::Grid2D &grid_, float score = 0);

    bool scanMatch(const Eigen::Matrix3f &predict_pose, const hjSlam_2d::PointCloud &input_source, const mapping::Grid2D &grid_, float score = 0);

    bool setRegistrationParam(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    inline Eigen::Matrix3f getFinalTransformation() { return final_transformation_ * predict_pose_; }; //final_transformation_
 
  private:
    size_t GetCorrespondence(
      const hjSlam_2d::PointCloud &input_source, 
      std::vector<Eigen::Vector2f> &xs,
      std::vector<Eigen::Vector2f> &ys
    );

    void GetTransform(
      const std::vector<Eigen::Vector2f> &xs,
      const std::vector<Eigen::Vector2f> &ys,
      Eigen::Matrix3f &transformation_
    );

    bool IsSignificant(
        const Eigen::Matrix3f &delta_transformation_,
        const float trans_eps);

    bool isBetterScore(const hjSlam_2d::PointCloud &points, const float score, const mapping::Grid2D &grid_);

    float max_corr_dist_;
    float trans_eps_; 
    float euc_fitness_eps_; 
    int max_iter_;
    float last_score_;

    // hjSlam_2d::PointCloud input_target_;
    std::vector<float> input_target_;

    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;
    std::unique_ptr<KDTree> input_target_kdtree_;

    hjSlam_2d::PointCloud last_points_;
    hjSlam_2d::PointCloud input_source_;
    // std::vector<float> input_source_;
    // std::unique_ptr<mapping::Grid2D> grid;
    mapping::scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;

    Eigen::Matrix3f transformation_;
    Eigen::Matrix3f final_transformation_;
    Eigen::Matrix3f predict_pose_;

};

}

#endif