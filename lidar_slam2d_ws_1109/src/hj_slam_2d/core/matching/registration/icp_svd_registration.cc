/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

// #include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

// #include "glog/logging.h"
#include "icp_svd_registration.h"
#include "../../../util/rigid_transform.h"
#include "../../../util/knn/knn_.h"

#include <ctime>
namespace hjSlam_2d
{
    ICPSVDRegistration::ICPSVDRegistration() /*: input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())*/
    {
        // parse params:
        // float max_corr_dist = node["max_corr_dist"].as<float>();
        // float trans_eps = node["trans_eps"].as<float>();
        // float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        // int max_iter = node["max_iter"].as<int>();

        float max_corr_dist = 0.01;
        float trans_eps = 0.0001;
        float euc_fitness_eps = 0.0001;
        int max_iter = 50;

        // setRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    ICPSVDRegistration::ICPSVDRegistration(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter) /*: input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())*/
    {
        // setRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
    }

    bool ICPSVDRegistration::setRegistrationParam(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter)
    {
        max_corr_dist_ = max_corr_dist;
        trans_eps_ = trans_eps;
        euc_fitness_eps_ = euc_fitness_eps;
        max_iter_ = max_iter;
        return true;
    }

    bool ICPSVDRegistration::setInputTarget(const hjSlam_2d::PointCloud &input_target)
    {
        // input_target_ = input_target;
        // std::vector<float> target_points;
        // time_t begin1, end1;
        for (auto item : input_target)
        {
            input_target_.push_back(item.position.x());
            input_target_.push_back(item.position.y());
            // input_target_.push_back(item.position.z());
        }

        input_target_kdtree_ = std::make_unique<KDTree>(input_target.size(), 2, input_target_.data(), 15);
        // begin1 = clock();
        input_target_kdtree_->index->buildIndex();
        // end1 = clock();
        // double ret = double(end1 - begin1) / 1000;
        // std::cout << "===========manual_kdtree_runtime:=============" << ret << "ms" << std::endl;
        // input_target_kdtree_->setInputCloud(input_target_);
        return true;
    }

    bool ICPSVDRegistration::scanMatch(const hjSlam_2d::PointCloud &input_source) //  输入待配准点云
    {
        last_points_ = input_source;
        // for (auto item : input_source)
        // {
        //     input_source_.push_back(item.position.x());
        //     input_source_.push_back(item.position.y());
        //     input_source_.push_back(item.position.z());
        // }
        // pre-process input source:
        // hjSlam_2d::PointCloud transformed_input_source(new CloudData::CLOUD());
        // pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

        // init estimation:
        transformation_.setIdentity();

        //
        // TODO: first option -- implement all computing logic on your own
        //
        // do estimation:
        int curr_iter = 0;
        while (curr_iter < max_iter_)
        {
            // 最大迭代次数
            // TODO: apply current estimation:
            // apply current estimation:
            hjSlam_2d::PointCloud curr_input_source;
            // TransformPointCloud(input_source, hjSlam_2d::transform::eigen_Matrix4dToRigid3d(transformation_.cast<double>()).cast<float>());

            for (int i = 0; i < last_points_.size(); ++i)
            {
                float new_x = last_points_[i].position.x() * transformation_(0, 0) + last_points_[i].position.y() * transformation_(0, 1) + transformation_(0, 2);
                float new_y = last_points_[i].position.x() * transformation_(1, 0) + last_points_[i].position.y() * transformation_(1, 1) + transformation_(1, 2);
                curr_input_source.push_back({Eigen::Vector3f(new_x, new_y, 0)});
            }
            last_points_ = curr_input_source;

            // pcl::transformPointCloud(*input_source_, *curr_input_source, transformation_);
            // TODO: get correspondence:
            std::vector<Eigen::Vector2f> xs;
            std::vector<Eigen::Vector2f> ys;

            // TODO: do not have enough correspondence -- break:
            int correspondenceN = GetCorrespondence(curr_input_source, xs, ys);
            if (correspondenceN < 3) //  寻找最邻近点的点对，当匹配点少于3个退出
                break;

            // std::cout << "=======correspondenceN========" << correspondenceN << std::endl;
            // TODO: update current transform:
            Eigen::Matrix3f delta_transformation;
            GetTransform(xs, ys, delta_transformation);

            // TODO: whether the transformation update is significant:
            if (!IsSignificant(delta_transformation, trans_eps_)) // 最大旋转矩阵
                break;
            // TODO: update transformation:
            transformation_ = delta_transformation * transformation_;

            ++curr_iter;
        }
        std::cout << "=========curr_iter===========" << curr_iter << std::endl;

        // set output:
        // result_pose = transformation_ * predict_pose;
        // result_pose = transformation_;

        // 归一化
        // Eigen::Quaternionf qr(result_pose.block<3, 3>(0, 0));
        // qr.normalize();
        // Eigen::Vector3f t = result_pose.block<3, 1>(0, 3);
        // result_pose.setIdentity();
        // result_pose.block<3, 3>(0, 0) = qr.toRotationMatrix();
        // result_pose.block<3, 1>(0, 3) = t;
        // pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

        // Eigen::Quaternionf qr(transformation_.block<3, 3>(0, 0));
        // qr.normalize();
        // Eigen::Vector3f t = transformation_.block<3, 1>(0, 3);
        // transformation_.setIdentity();
        // transformation_.block<3, 3>(0, 0) = qr.toRotationMatrix();
        // transformation_.block<3, 1>(0, 3) = t;

        return true;
    }

    bool ICPSVDRegistration::scanMatch(const hjSlam_2d::PointCloud &input_source, const mapping::Grid2D &grid_, float score)
    {
        // input_source_ = input_source;
        last_points_ = input_source;
        // init estimation:
        transformation_.setIdentity();

        int curr_iter = 0;
        while (curr_iter < max_iter_)
        {
            hjSlam_2d::PointCloud curr_input_source;
            for (int i = 0; i < last_points_.size(); ++i)
            {
                float new_x = last_points_[i].position.x() * transformation_(0, 0) + last_points_[i].position.y() * transformation_(0, 1) + transformation_(0, 2);
                float new_y = last_points_[i].position.x() * transformation_(1, 0) + last_points_[i].position.y() * transformation_(1, 1) + transformation_(1, 2);
                curr_input_source.push_back({Eigen::Vector3f(new_x, new_y, 0)});
            }
            last_points_ = curr_input_source;

            // TODO: get correspondence:
            std::vector<Eigen::Vector2f> xs;
            std::vector<Eigen::Vector2f> ys;

            // TODO: do not have enough correspondence -- break:
            int correspondenceN = GetCorrespondence(curr_input_source, xs, ys);
            if (correspondenceN < 3) //  寻找最邻近点的点对，当匹配点少于3个退出
                break;

            // TODO: update current transform:
            Eigen::Matrix3f delta_transformation;
            GetTransform(xs, ys, delta_transformation);

            if (!IsSignificant(delta_transformation, trans_eps_))
                break;
            // TODO: whether the transformation update is significant:
            // 最大旋转矩阵

            // TODO: update transformation:
            transformation_ = delta_transformation * transformation_;

            ++curr_iter;
        }
        std::cout << "=========curr_iter===========" << curr_iter << std::endl;
        return true;
    }

    bool ICPSVDRegistration::scanMatch(const Eigen::Matrix3f &predict_pose, const hjSlam_2d::PointCloud &input_source, const mapping::Grid2D &grid_, float score)
    {
        // init estimation:
        last_score_ = score;
        predict_pose_ = predict_pose;
        transformation_.setIdentity();
        final_transformation_.setIdentity();

        for (int i = 0; i < input_source.size(); ++i) {
            float new_x = predict_pose(0, 0) * input_source[i].position.x() + predict_pose(0, 1) * input_source[i].position.y() + predict_pose(0, 2);
            float new_y = predict_pose(1, 0) * input_source[i].position.x() + predict_pose(1, 1) * input_source[i].position.y() + predict_pose(1, 2);
            last_points_.push_back({Eigen::Vector3f(new_x, new_y, 0)});
        }

        int curr_iter = 0;
        while (curr_iter < max_iter_){
            hjSlam_2d::PointCloud cur_input_source;
            for (int i = 0; i < last_points_.size(); ++i) {
                float new_x = last_points_[i].position.x() * transformation_(0, 0) + last_points_[i].position.y() * transformation_(0, 1) + transformation_(0, 2);
                float new_y = last_points_[i].position.x() * transformation_(1, 0) + last_points_[i].position.y() * transformation_(1, 1) + transformation_(1, 2);
                cur_input_source.push_back({Eigen::Vector3f(new_x, new_y, 0)});
            }
            last_points_ = cur_input_source; // TODO: lz, 这里是不是有问题，每次回push_back input_source的内容，

            // isBetterScore(last_points_, last_score_, grid_);

            // TODO: get correspondence:
            std::vector<Eigen::Vector2f> xs;
            std::vector<Eigen::Vector2f> ys;

            // TODO: do not have enough correspondence -- break:
            int correspondenceN = GetCorrespondence(cur_input_source, xs, ys);
            if (correspondenceN < 3) //  寻找最邻近点的点对，当匹配点少于3个退出
                break;

            // TODO: update current transform:
            Eigen::Matrix3f delta_transformation;
            GetTransform(xs, ys, delta_transformation);

            // TODO: whether the transformation update is significant:
            // 最大旋转矩阵
            if (IsSignificant(delta_transformation, trans_eps_))
                break; 

            // TODO: update transformation:
            transformation_ = delta_transformation * transformation_;

            ++curr_iter;
        }

        // final_transformation_ = final_transformation_ * predict_pose;
        std::cout << "=========curr_iter===========" << curr_iter << std::endl;
        return true;
    }

    size_t ICPSVDRegistration::GetCorrespondence(
        const hjSlam_2d::PointCloud &input_source,
        std::vector<Eigen::Vector2f> &xs,
        std::vector<Eigen::Vector2f> &ys)
    {
        const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;
        size_t num_corr = 0;

        // TODO: set up point correspondence
        std::vector<long> corr_ind(1); // index
        std::vector<float> corr_sq_dis; // correspondence_square_dis
        float sum_dis = 0;

        /* test knn
                std::vector<float> source_pt{input_source[0].position.x(), input_source[0].position.y()};
                cpp_knn_omp_kdtree(input_target_.data(), input_target_.size() / 2, 2, source_pt.data(), 1, 10, corr_ind.data(), *input_target_kdtree_, corr_sq_dis);
                for (auto it : corr_ind)
                    std::cout << "svd_knn_indice:" << it << std::endl;
                for (auto it : corr_sq_dis)
                    std::cout << "svd_knn_dis:" << it << std::endl;
        */

        for (size_t i = 0; i < input_source.size(); ++i)
        {
            std::vector<float> source_pt{input_source[i].position.x(), input_source[i].position.y()};
            cpp_knn_omp_kdtree(input_target_.data(), input_target_.size() / 2, 2, source_pt.data(), 1, 1, corr_ind.data(), *input_target_kdtree_, corr_sq_dis);

            float dis = corr_sq_dis.at(0);
            if (dis > MAX_CORR_DIST_SQR)
                continue;

            sum_dis += dis;
            // add  correspondence:
            Eigen::Vector2f x(
                input_target_[corr_ind.at(0) * 2],
                input_target_[corr_ind.at(0) * 2 + 1]);

            Eigen::Vector2f y(
                input_source[i].position.x(),
                input_source[i].position.y());

            xs.push_back(x);
            ys.push_back(y);

            ++num_corr;
        }
        // std::cout << "===============average_dis===================" << sum_dis / num_corr << std::endl;

        return num_corr;
    }

    void ICPSVDRegistration::GetTransform(
        const std::vector<Eigen::Vector2f> &xs,
        const std::vector<Eigen::Vector2f> &ys,
        Eigen::Matrix3f &delta_transformation_)
    {
        const size_t N = xs.size();
        // find centroids of mu_x and mu_y:
        Eigen::Vector2f mu_x = Eigen::Vector2f::Zero();
        Eigen::Vector2f mu_y = Eigen::Vector2f::Zero();
        for (size_t i = 0; i < N; ++i)
        {
            mu_x += xs.at(i);
            mu_y += ys.at(i);
        }
        mu_x /= N;
        mu_y /= N;

        // build H:
        Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
        for (size_t i = 0; i < N; ++i){
            H += (ys.at(i) - mu_y) * (xs.at(i) - mu_x).transpose();
        }

        float angle = std::atan2((H(0, 1) - H(1, 0)), (H(0, 0) + H(1, 1)));
        Eigen::Matrix<float, 2, 2> R(Eigen::Matrix<float, 2, 2>::Identity());
        R(0, 0) = R(1, 1) = std::cos(angle);
        R(0, 1) = -std::sin(angle);
        R(1, 0) = std::sin(angle);

        // Return the correct transformation
        // transformation_matrix.topLeftCorner(3, 3).matrix() = R;
        // const Eigen::Matrix<float, 3, 1> Rc(R * centroid_src.head(3).matrix());
        // transformation_matrix.block(0, 3, 3, 1).matrix() = centroid_tgt.head(3) - Rc;
        // solve R:
        // Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Eigen::Matrix<float, 2, 2> u = svd.matrixU();
        // Eigen::Matrix<float, 2, 2> v = svd.matrixV();
        // Eigen::Matrix<float, 2, 2> R = v * u.transpose();
        // double angle = std::atan2(R(1, 0), R(0, 0));
        // std::cout << "================angle" << angle << std::endl;

        // solve t:
        Eigen::Vector2f t = mu_x - R * mu_y;
        // set output:
        delta_transformation_.setIdentity();
        delta_transformation_.block<2, 2>(0, 0) = R;
        delta_transformation_.block<2, 1>(0, 2) = t;

    }

    bool ICPSVDRegistration::IsSignificant(
        const Eigen::Matrix3f &delta_transformation_,
        const float trans_eps)
    {
        // a. translation magnitude -- norm:
        // float translation_magnitude = transformation.block<2, 1>(0, 2).norm();
        // b. rotation magnitude -- angle:
        // float rotation_magnitude = fabs(acos((transformation.block<2, 2>(0, 0).trace() - 1.0f) / 2.0f));

        // double angle = std::atan2(transformation(1, 0), transformation(0, 0));
        // std::cout <<"trans_eps:" << trans_eps <<" " << "translation_magnitude:" << delta_transformation_.block<2, 1>(0, 2).x() << " " <<  delta_transformation_.block<2, 1>(0, 2).y()<< std::endl;
        if (abs(delta_transformation_.block<2, 1>(0, 2).x()) < trans_eps && (abs(delta_transformation_.block<2, 1>(0, 2).y()) < trans_eps))
            return true;
        else
            return false;
    }

    bool ICPSVDRegistration::isBetterScore(const hjSlam_2d::PointCloud &points, const float score, const mapping::Grid2D &grid_)
    {
        const double icp_score = real_time_correlative_scan_matcher_.compute_occupied_score(points, grid_);
        if (icp_score > last_score_)
        {
            final_transformation_ = transformation_;
            last_score_ = icp_score;
        }
            // std::cout << "===============better_icp_score:" << icp_score << std::endl;
    }


} // namespace lidar_localization