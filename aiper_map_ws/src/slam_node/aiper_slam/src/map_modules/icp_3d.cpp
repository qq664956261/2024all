
#include "map_modules/ExtrinsicErrorTerm/icp_3d.h"
#include "map_modules/ExtrinsicErrorTerm/common/math_utils.h"
#include "log.h"

namespace sad
{

    bool Icp3d::AlignP2P(SE3 &init_pose)
    {
        // LOG(INFO) << "aligning with point to point";
        assert(target_ != nullptr && source_ != nullptr);

        SE3 pose = init_pose;
        if (!options_.use_initial_translation_)
        {
            pose.translation() = target_center_ - source_center_; // 设置平移初始值
        }

        // 对点的索引，预先生成
        std::vector<int> index(source_->points.size());
        for (std::size_t i = 0; i < index.size(); ++i)
        {
            index[i] = i;
        }

        // 我们来写一些并发代码
        std::vector<bool> effect_pts(index.size(), false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
        std::vector<Vec3d> errors(index.size());

        for (int iter = 0; iter < options_.max_iteration_; ++iter)
        {
            Mat6d H_sum = Mat6d::Zero(); // 初始化Hessian矩阵为零矩阵
            Vec6d b_sum = Vec6d::Zero(); // 初始化误差向量为零向量
            double total_res = 0;
            int effective_num = 0;
            std::pair<Mat6d, Vec6d> H_and_err;
            // 使用传统的for循环遍历所有索引
            for (size_t i = 0; i < index.size(); ++i)
            {
                int idx = index[i];
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = pose * q; // 转换之后的q
                std::vector<int> nn;
                kdtree_->GetClosestPoint(ToPointType(qs), nn, 1);

                if (!nn.empty())
                {
                    Vec3d p = ToVec3d(target_->points[nn[0]]);
                    double dis2 = (p - qs).squaredNorm();
                    if (dis2 > options_.max_nn_distance_)
                    {
                        // 点离的太远了不要
                        effect_pts[idx] = false;
                        continue;
                    }

                    effect_pts[idx] = true;

                    // 构建残差
                    Vec3d e = p - qs;
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = -Mat3d::Identity();

                    jacobians[idx] = J;
                    errors[idx] = e;

                    // 计算Hessian矩阵和error向量
                    H_sum += J.transpose() * J;
                    b_sum -= J.transpose() * e;

                    // 累加总残差和有效点数
                    total_res += e.dot(e);
                    effective_num++;
                }
                else
                {
                    effect_pts[idx] = false;
                }
            }
            H_and_err = std::make_pair(H_sum, b_sum);

            if (effective_num < options_.min_effective_pts_)
            {
                // LOG(WARNING) << "effective num too small: " << effective_num;
                return false;
            }

            Mat6d H = H_and_err.first;
            Vec6d err = H_and_err.second;

            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            // 更新
            // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
            //   << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
                HJ_INFO("iter = %d, pose_error = %f", iter, pose_error);
                // LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            }

            if (dx.norm() < options_.eps_)
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        return true;
    }

    bool Icp3d::AlignP2Plane(SE3 &init_pose)
    {
        // LOG(INFO) << "aligning with point to plane";
        assert(target_ != nullptr && source_ != nullptr);
        // 整体流程与p2p一致，读者请关注变化部分

        SE3 pose = init_pose;
        if (!options_.use_initial_translation_)
        {
            pose.translation() = target_center_ - source_center_; // 设置平移初始值
        }

        std::vector<int> index(source_->points.size());
        for (std::size_t i = 0; i < index.size(); ++i)
        {
            index[i] = i;
        }

        std::vector<bool> effect_pts(index.size(), false);
        std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
        std::vector<double> errors(index.size());

        for (int iter = 0; iter < options_.max_iteration_; ++iter)
        {
            Mat6d H_sum = Mat6d::Zero(); // 初始化Hessian矩阵为零矩阵
            Vec6d b_sum = Vec6d::Zero(); // 初始化误差向量为零向量
            double total_res = 0.0;
            int effective_num = 0;
            std::pair<Mat6d, Vec6d> H_and_err;
            for (size_t i = 0; i < index.size(); ++i)
            {
                int idx = index[i];
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = pose * q; // 转换之后的q
                std::vector<int> nn;
                kdtree_->GetClosestPoint(ToPointType(qs), nn, 5); // 这里取5个最近邻

                if (nn.size() > 3)
                {
                    std::vector<Vec3d> nn_eigen;
                    for (size_t j = 0; j < nn.size(); ++j)
                    {
                        nn_eigen.emplace_back(ToVec3d(target_->points[nn[j]]));
                    }

                    Vec4d n;
                    if (!math::FitPlane(nn_eigen, n))
                    {
                        // 失败的不要
                        effect_pts[idx] = false;
                        continue;
                    }

                    double dis = n.head<3>().dot(qs) + n[3];
                    if (fabs(dis) > options_.max_plane_distance_)
                    {
                        // 点离的太远了不要
                        effect_pts[idx] = false;
                        continue;
                    }

                    effect_pts[idx] = true;

                    // 构建残差
                    Eigen::Matrix<double, 1, 6> J;
                    J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);
                    J.block<1, 3>(0, 3) = n.head<3>().transpose();

                    // 更新雅可比和误差
                    jacobians[idx] = J;
                    errors[idx] = dis;

                    // 累加Hessian和误差向量
                    H_sum += J.transpose() * J;
                    b_sum -= J.transpose() * dis;

                    // 累加总残差和有效点数
                    total_res += dis * dis;
                    effective_num++;
                }
                else
                {
                    effect_pts[idx] = false;
                }
            }
            H_and_err = std::make_pair(H_sum, b_sum);

            if (effective_num < options_.min_effective_pts_)
            {
                // LOG(WARNING) << "effective num too small: " << effective_num;
                return false;
            }

            Mat6d H = H_and_err.first;
            Vec6d err = H_and_err.second;

            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            // 更新
            // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
            //   << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
                HJ_INFO("iter = %d, pose_error = %f", iter, pose_error);
                // LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            }

            if (dx.norm() < options_.eps_)
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        return true;
    }

    void Icp3d::BuildTargetKdTree()
    {
        kdtree_ = std::make_shared<KdTree>();
        kdtree_->BuildTree(target_);
        kdtree_->SetEnableANN();
    }

    bool Icp3d::AlignP2Line(SE3 &init_pose)
    {

        // LOG(INFO) << "aligning with point to line";
        assert(target_ != nullptr && source_ != nullptr);
        // 点线与点面基本是完全一样的

        SE3 pose = init_pose;
        if (options_.use_initial_translation_)
        {
            pose.translation() = target_center_ - source_center_; // 设置平移初始值
            // LOG(INFO) << "init trans set to " << pose.translation().transpose();
        }

        std::vector<int> index(source_->points.size());
        for (std::size_t i = 0; i < index.size(); ++i)
        {
            index[i] = i;
        }

        std::vector<bool> effect_pts(index.size(), false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
        std::vector<Vec3d> errors(index.size());

        for (int iter = 0; iter < options_.max_iteration_; ++iter)
        {

            Mat6d H_sum = Mat6d::Zero(); // 初始化Hessian矩阵为零矩阵
            Vec6d b_sum = Vec6d::Zero(); // 初始化误差向量为零向量
            double total_res = 0.0;
            int effective_num = 0;
            std::pair<Mat6d, Vec6d> H_and_err;
            for (size_t i = 0; i < index.size(); ++i)
            {
                int idx = index[i];
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = pose * q; // 转换之后的q
                std::vector<int> nn;
                kdtree_->GetClosestPoint(ToPointType(qs), nn, 5); // 这里取5个最近邻

                if (nn.size() == 5)
                {
                    // 转换到Eigen格式
                    std::vector<Vec3d> nn_eigen;
                    for (size_t j = 0; j < nn.size(); ++j)
                    {
                        nn_eigen.emplace_back(ToVec3d(target_->points[nn[j]]));
                    }

                    Vec3d d, p0;
                    if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_))
                    {
                        // 如果拟合失败，这个点不予考虑
                        effect_pts[idx] = false;
                        continue;
                    }

                    // 计算点到直线的距离
                    Vec3d err = SO3::hat(d) * (qs - p0);

                    if (err.norm() > options_.max_line_distance_)
                    {
                        // 如果点离直线太远，不考虑此点
                        effect_pts[idx] = false;
                        continue;
                    }

                    // 标记点为有效，并构建雅可比矩阵
                    effect_pts[idx] = true;
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = SO3::hat(d);

                    // 计算并累加雅可比矩阵和误差
                    H_sum += J.transpose() * J;
                    b_sum -= J.transpose() * err;

                    // 累加总残差和有效点数
                    total_res += err.dot(err);
                    effective_num++;
                }
                else
                {
                    effect_pts[idx] = false;
                }
            }
            H_and_err = std::make_pair(H_sum, b_sum);

            if (effective_num < options_.min_effective_pts_)
            {
                // LOG(WARNING) << "effective num too small: " << effective_num;
                return false;
            }
//
            Mat6d H = H_and_err.first;
            Vec6d err = H_and_err.second;

            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
                HJ_INFO("iter = %d, pose_error = %f", iter, pose_error);
                // LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            }

            // 更新
            // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
            //   << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

            if (dx.norm() < options_.eps_)
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        return true;
    }

    double Icp3d::GetFitnessScore(SE3 &result)
    {
      if (source_ == nullptr || target_ == nullptr)
      {
        // LOG(WARNING) << "Source or target point cloud is nullptr.";
        return -1; // 返回-1作为错误代码
      }

      double total_distance = 0.0;
      int count = 0; // 计算有效配对的点数
      for (const auto &src_point : source_->points)
      {
        Vec3d q = ToVec3d(src_point);
        q = result * q; // 转换之后的q
        std::vector<int> nn_indices;                             // 用于存储最近邻的索引
        kdtree_->GetClosestPoint(ToPointType(q), nn_indices, 1); // 获取最近的一个点

        if (!nn_indices.empty())
        {
          auto closest_point = ToVec3d(target_->points[nn_indices[0]]);
          double distance = (q - closest_point).norm();

          if (distance <= options_.max_nn_distance_)
          { // 只计算距离小于某个阈值的点对
            total_distance += distance;
            count++;
          }
        }
      }

      if (count == 0)
        return -1;                 // 如果没有有效的点对，返回错误代码

      return total_distance / count; // 返回平均距离作为适应性评分
    }


} // namespace aiper_relocalization_ns