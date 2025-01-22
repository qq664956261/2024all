
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_

// #include <pcl/registration/icp.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include "registration_interface.h"
#include "../../../third_party/sophus/se3.hpp" // 添加 se3
#include "../../../util/knn/KDTreeTableAdaptor.h"
#include "../../../util/point_cloud.h"
#include <memory>

// create the kdtree
typedef KDTreeTableAdaptor<float, float> KDTree;

namespace hjSlam_2d
{
    class ICPRegistrationManual : public RegistrationInterface
    {
    public:
        ICPRegistrationManual();
        ICPRegistrationManual(float max_correspond_dis, int max_iter);

        bool setInputTarget(const hjSlam_2d::PointCloud &input_target) override;
        bool scanMatch(const hjSlam_2d::PointCloud &input_source) override;

        inline  Eigen::Matrix4f getFinalTransformation(){ return transformation_;};

        bool SetRegistrationParam(float max_correspond_dis, int max_iter);
    private:
        void calculateTrans(const hjSlam_2d::PointCloud &input_cloud); // 计算旋转矩阵

    private:
        //   hjSlam_2d::PointCloud target_cloud_;
        std::vector<float> input_target_;
        //   pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;
        float max_correspond_distance_ = 0.01; // 阈值
        int max_iterator_ = 50;                // 最大迭代次数
        hjSlam_2d::PointCloud last_points_;

        Eigen::Matrix3f rotation_matrix_; // 旋转矩阵
        Eigen::Vector3f translation_;     // 平移矩阵
        Eigen::Matrix4f transformation_;  // 转换矩阵

        std::unique_ptr<KDTree> input_target_kdtree_;
    };
}

#endif
