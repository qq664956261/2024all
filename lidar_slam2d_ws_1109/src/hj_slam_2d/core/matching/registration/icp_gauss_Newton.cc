
#include "icp_gauss_Newton.h"
// #include "glog/logging.h"
#include <Eigen/Dense>
#include "../../../util/rigid_transform.h"
#include "../../../util/knn/knn_.h"

namespace hjSlam_2d {

ICPRegistrationManual::ICPRegistrationManual(){
}

ICPRegistrationManual ::ICPRegistrationManual(float max_correspond_dis,
                                              int max_iter)
{
     SetRegistrationParam(max_correspond_dis, max_iter);
}

bool  ICPRegistrationManual::SetRegistrationParam(float max_correspond_dis,
                                                                                                            int  max_iter){
    max_correspond_distance_  = max_correspond_dis;
    max_iterator_ =  max_iter;
    // LOG(INFO)     <<  "ICP Manual  的匹配参数为 :   "   << std::endl
    //                            << "max_correspond_dis:  "       <<  max_correspond_dis  << ", " 
    //                            << "max_iter:  "      <<  max_iter   <<  std::endl
    //                            << std :: endl;
    return true;
}

bool ICPRegistrationManual::setInputTarget(
    const hjSlam_2d::PointCloud &input_target)
{
    for (auto item : input_target)
    {
        input_target_.push_back(item.position.x());
        input_target_.push_back(item.position.y());
        input_target_.push_back(item.position.z());
    }

    input_target_kdtree_ = std::make_unique<KDTree>(input_target.size(), 3, input_target_.data(), 10);
    input_target_kdtree_->index->buildIndex();
    // input_target_kdtree_->setInputCloud(input_target_);
    return true;

}

bool ICPRegistrationManual::scanMatch(const hjSlam_2d::PointCloud &input_source)
{
        // transformation_ = predict_pose;
        transformation_.setIdentity();
        rotation_matrix_ = transformation_.block<3, 3>(0, 0); // 取旋转矩阵
        translation_ = transformation_.block<3, 1>(0, 3);     // 取平移矩阵

        calculateTrans(input_source); // 计算变换矩阵

        // pcl::transformPointCloud(*input_source,   *result_cloud_ptr,  transformation_);   // 对点云进行变换
        // result_pose = transformation_;

        return true;
}

void ICPRegistrationManual::calculateTrans(const hjSlam_2d::PointCloud   &input_source){

    hjSlam_2d::PointCloud  transformed_cloud;
    int knn = 1;     // 进行 1nn的搜索
    int iterator_num = 0;
    last_points_ = input_source;

    while(iterator_num < max_iterator_)
    {
        // pcl::transformPointCloud(*input_source,*transformed_cloud,transformation_);    // 对点云进行变换
        // TransformPointCloud(input_source, hjSlam_2d::transform::eigen_Matrix4dToRigid3d(transformation_.cast<double>()).cast<float>());
        hjSlam_2d::PointCloud curr_input_source;
        for (int i = 0; i < last_points_.size(); ++i)
        {
            float new_x = last_points_[i].position.x() * transformation_(0, 0) + last_points_[i].position.y() * transformation_(0, 1) + transformation_(0, 3);
            float new_y = last_points_[i].position.x() * transformation_(1, 0) + last_points_[i].position.y() * transformation_(1, 1) + transformation_(1, 3);
            curr_input_source.push_back({Eigen::Vector3f(new_x, new_y, 0)});

        }
        last_points_ = curr_input_source;

        Eigen::Matrix<float,6,6> Hessian;
        Eigen::Matrix<float,6,1>B;
        Hessian.setZero();
        B.setZero();     // 归零

        for(size_t i =0; i < curr_input_source.size();  ++i)
        {
            auto ori_point = curr_input_source[i];
            // if(!pcl::isFinite(ori_point))
            //     continue;
            auto transformed_point = curr_input_source[i];
            std::vector<long> corr_ind(1);  // index
            std::vector<float> corr_sq_dis; // correspondence_square_dis
            // kdtree_ptr_->nearestKSearch(transformed_point,knn,indexs,distances);      // knn搜索
            std::vector<float> source_pt{input_source[i].position.x(), input_source[i].position.y()};

            cpp_knn_omp_kdtree(input_target_.data(), input_target_.size() / 3, 3, source_pt.data(), 1, 1, corr_ind.data(), *input_target_kdtree_, corr_sq_dis);

            if (corr_sq_dis[0] > max_correspond_distance_)
            {
                continue;
            }
            Eigen::Vector3f closet_point = Eigen::Vector3f(input_target_[corr_ind.at(0) * 3],
                                                           input_target_[corr_ind.at(0) * 3 + 1],
                                                           input_target_[corr_ind.at(0) * 3 + 2]);

            // 计算 原始点 与  最邻近点 的 距离
            Eigen::Vector3f err_dis =
                Eigen::Vector3f(source_pt[0], source_pt[1], source_pt[2]) - closet_point;

            Eigen::Matrix<float,3,6> Jacobian(Eigen::Matrix<float,3,6>::Zero());
            Jacobian.leftCols<3>() = Eigen::Matrix3f::Identity();
            Jacobian.rightCols<3>() = 
                    -rotation_matrix_* Sophus::SO3f::hat(Eigen::Vector3f(ori_point.position.x(),ori_point.position.y(),ori_point.position.z())) ;
                    Hessian  +=  Jacobian.transpose()* Jacobian; 
                    B += -Jacobian.transpose()*err_dis;
        }
        iterator_num++;
        if(Hessian.determinant() == 0)
        {
            continue;
        }
        Eigen::Matrix<float,6,1> delta_x =  Hessian.inverse()*B;

        translation_ += delta_x.head<3>();
        auto  delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
        // rotation_matrix_ *= delta_rotation.matrix();
        rotation_matrix_  = delta_rotation.matrix() * rotation_matrix_;

        transformation_.block<3,3>(0,0) = rotation_matrix_;
        transformation_.block<3,1>(0,3) = translation_;

    }

    std::cout << "=============iterator_num=================" << iterator_num << std::endl;
}
}
