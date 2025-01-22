
#include "relocalization/icp_3d.h"
#include "relocalization/common/math_utils.h"

namespace aiper_relocalization_ns
{
    bool Icp3d::AlignP2PSIM3(SE3 &init_pose, double &scale)
    {
        assert(target_ != nullptr && source_ != nullptr);

        SE3 pose = init_pose;
        double s = scale;
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
        std::vector<Eigen::Matrix<double, 3, 7>> jacobians(index.size());
        std::vector<Vec3d> errors(index.size());

        for (int iter = 0; iter < options_.max_iteration_; ++iter)
        {
            Mat7d H_sum = Mat7d::Zero(); // 初始化Hessian矩阵为零矩阵
            Vec7d b_sum = Vec7d::Zero(); // 初始化误差向量为零向量
            double total_res = 0;
            int effective_num = 0;
            std::pair<Mat7d, Vec7d> H_and_err;
            // 使用传统的for循环遍历所有索引
            for (size_t i = 0; i < index.size(); ++i)
            {
                int idx = index[i];
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = s * pose.so3().matrix() * q + pose.translation(); // 转换之后的q
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
                    Eigen::Matrix<double, 3, 7> J;
                    J.block<3, 3>(0, 0) = s * pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = -Mat3d::Identity();
                    J.block<3, 1>(0, 6) = pose.so3().matrix() * q;

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
                return false;
            }

            Mat7d H = H_and_err.first;
            Vec7d err = H_and_err.second;

            Vec7d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.segment<3>(3);
            s += dx[6];

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
            }

            if (dx.norm() < options_.eps_)
            {
                break;
            }
        }

        init_pose = pose;
        scale = s;
        sim3_s_ = s;
        result = pose;
        return true;
    }

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
            // // gauss-newton 迭代
            // // 最近邻，可以并发
            // std::for_each( index.begin(), index.end(), [&](int idx) {
            //     auto q = ToVec3d(source_->points[idx]);
            //     Vec3d qs = pose * q;  // 转换之后的q
            //     std::vector<int> nn;
            //     kdtree_->GetClosestPoint(ToPointType(qs), nn, 1);

            //     if (!nn.empty()) {
            //         Vec3d p = ToVec3d(target_->points[nn[0]]);
            //         double dis2 = (p - qs).squaredNorm();
            //         if (dis2 > options_.max_nn_distance_) {
            //             // 点离的太远了不要
            //             effect_pts[idx] = false;
            //             return;
            //         }

            //         effect_pts[idx] = true;

            //         // build residual
            //         Vec3d e = p - qs;
            //         Eigen::Matrix<double, 3, 6> J;
            //         J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(q);
            //         J.block<3, 3>(0, 3) = -Mat3d::Identity();

            //         jacobians[idx] = J;
            //         errors[idx] = e;
            //     } else {
            //         effect_pts[idx] = false;
            //     }
            // });

            // // 累加Hessian和error,计算dx
            // // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
            // double total_res = 0;
            // int effective_num = 0;
            // auto H_and_err = std::accumulate(
            //     index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            //     [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre,
            //                                                                    int idx) -> std::pair<Mat6d, Vec6d> {
            //         if (!effect_pts[idx]) {
            //             return pre;
            //         } else {
            //             total_res += errors[idx].dot(errors[idx]);
            //             effective_num++;
            //             return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
            //                                            pre.second - jacobians[idx].transpose() * errors[idx]);
            //         }
            //     });
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
                // LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            }

            if (dx.norm() < options_.eps_)
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        result = pose;
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
            // gauss-newton 迭代
            // 最近邻，可以并发
            // std::for_each(index.begin(), index.end(), [&](int idx)
            //               {
            // auto q = ToVec3d(source_->points[idx]);
            // Vec3d qs = pose * q;  // 转换之后的q
            // std::vector<int> nn;
            // kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 这里取5个最近邻
            // if (nn.size() > 3) {
            //     // convert to eigen
            //     std::vector<Vec3d> nn_eigen;
            //     for (std::size_t i = 0; i < nn.size(); ++i) {
            //         nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
            //     }

            //     Vec4d n;
            //     if (!math::FitPlane(nn_eigen, n)) {
            //         // 失败的不要
            //         effect_pts[idx] = false;
            //         return;
            //     }

            //     double dis = n.head<3>().dot(qs) + n[3];
            //     if (fabs(dis) > options_.max_plane_distance_) {
            //         // 点离的太远了不要
            //         effect_pts[idx] = false;
            //         return;
            //     }

            //     effect_pts[idx] = true;

            //     // build residual
            //     Eigen::Matrix<double, 1, 6> J;
            //     J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);
            //     J.block<1, 3>(0, 3) = n.head<3>().transpose();

            //     jacobians[idx] = J;
            //     errors[idx] = dis;
            // } else {
            //     effect_pts[idx] = false;
            // } });

            // // 累加Hessian和error,计算dx
            // // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
            // double total_res = 0;
            // int effective_num = 0;
            // auto H_and_err = std::accumulate(
            //     index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            //     [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d> &pre,
            //                                                                    int idx) -> std::pair<Mat6d, Vec6d>
            //     {
            //         if (!effect_pts[idx])
            //         {
            //             return pre;
            //         }
            //         else
            //         {
            //             total_res += errors[idx] * errors[idx];
            //             effective_num++;
            //             return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
            //                                            pre.second - jacobians[idx].transpose() * errors[idx]);
            //         }
            //     });
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
                // LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            }

            if (dx.norm() < options_.eps_)
            {
                // LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        result = pose;
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
        if (!options_.use_initial_translation_)
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
            // // gauss-newton 迭代
            // // 最近邻，可以并发
            // std::for_each(index.begin(), index.end(), [&](int idx)
            //               {
            // auto q = ToVec3d(source_->points[idx]);
            // Vec3d qs = pose * q;  // 转换之后的q
            // std::vector<int> nn;
            // kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 这里取5个最近邻
            // if (nn.size() == 5) {
            //     // convert to eigen
            //     std::vector<Vec3d> nn_eigen;
            //     for (std::size_t i = 0; i < 5; ++i) {
            //         nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
            //     }

            //     Vec3d d, p0;
            //     if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
            //         // 失败的不要
            //         effect_pts[idx] = false;
            //         return;
            //     }

            //     Vec3d err = SO3::hat(d) * (qs - p0);// 计算点到直线的距离

            //     if (err.norm() > options_.max_line_distance_) {
            //         // 点离的太远了不要
            //         effect_pts[idx] = false;
            //         return;
            //     }

            //     effect_pts[idx] = true;

            //     // build residual
            //     Eigen::Matrix<double, 3, 6> J;
            //     J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.so3().matrix() * SO3::hat(q);
            //     J.block<3, 3>(0, 3) = SO3::hat(d);

            //     jacobians[idx] = J;
            //     errors[idx] = err;
            // } else {
            //     effect_pts[idx] = false;
            // } });

            // // 累加Hessian和error,计算dx
            // // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
            // double total_res = 0;
            // int effective_num = 0;
            // auto H_and_err = std::accumulate(
            //     index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            //     [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d> &pre,
            //                                                                    int idx) -> std::pair<Mat6d, Vec6d>
            //     {
            //         if (!effect_pts[idx])
            //         {
            //             return pre;
            //         }
            //         else
            //         {
            //             total_res += errors[idx].dot(errors[idx]);
            //             effective_num++;
            //             return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
            //                                            pre.second - jacobians[idx].transpose() * errors[idx]);
            //         }
            //     });
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
                return false;
            }

            Mat6d H = H_and_err.first;
            Vec6d err = H_and_err.second;

            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
            }

            if (dx.norm() < options_.eps_)
            {
                break;
            }
        }

        init_pose = pose;
        result = pose;
        return true;
    }

    bool Icp3d::AlignP2LineSIM3(SE3 &init_pose, double &scale)
    {
        assert(target_ != nullptr && source_ != nullptr);
        // 点线与点面基本是完全一样的

        SE3 pose = init_pose;
        double s = scale;
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
        std::vector<Eigen::Matrix<double, 3, 7>> jacobians(index.size());
        std::vector<Vec3d> errors(index.size());

        for (int iter = 0; iter < options_.max_iteration_; ++iter)
        {
            Mat7d H_sum = Mat7d::Zero(); // 初始化Hessian矩阵为零矩阵
            Vec7d b_sum = Vec7d::Zero(); // 初始化误差向量为零向量
            double total_res = 0.0;
            int effective_num = 0;
            std::pair<Mat7d, Vec7d> H_and_err;
            for (size_t i = 0; i < index.size(); ++i)
            {
                int idx = index[i];
                auto q = ToVec3d(source_->points[idx]);
                Vec3d qs = s * pose.so3().matrix() * q + pose.translation(); // 转换之后的q
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
                    Eigen::Matrix<double, 3, 7> J;
                    J.block<3, 3>(0, 0) = -s * SO3::hat(d) * pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = SO3::hat(d);
                    J.block<3, 1>(0, 6) = SO3::hat(d) * pose.so3().matrix() * q;

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
                return false;
            }

            Mat7d H = H_and_err.first;
            Vec7d err = H_and_err.second;

            Vec7d dx = H.inverse() * err;

            if (std::isnan(dx[0]) || std::isnan(dx[1]) || std::isnan(dx[2]) || std::isnan(dx[3]) || std::isnan(dx[4]) || std::isnan(dx[5]) || std::isnan(dx[6]))
            {
                break;
            }
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.segment<3>(3);
            s += dx[6];

            if (gt_set_)
            {
                double pose_error = (gt_pose_.inverse() * pose).log().norm();
            }

            if (dx.norm() < options_.eps_)
            {
                break;
            }
        }

        init_pose = pose;
        scale = s;
        sim3_s_ = s;
        result = pose;
        return true;
    }

    double Icp3d::GetFitnessScore()
    {
        if (source_ == nullptr || target_ == nullptr)
        {
            // LOG(WARNING) << "Source or target point cloud is nullptr.";
            return 100; // 返回-1作为错误代码
        }

        double total_distance = 0.0;
        int count = 0; // 计算有效配对的点数
        for (const auto &src_point : source_->points)
        {
            Vec3d q = ToVec3d(src_point);
            q = result * q;                                          // 转换之后的q
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
            return 100;                // 如果没有有效的点对，返回错误代码
        return total_distance / count; // 返回平均距离作为适应性评分
    }

    double Icp3d::GetFitnessScoreSIM3()
    {
        if (source_ == nullptr || target_ == nullptr)
        {
            // LOG(WARNING) << "Source or target point cloud is nullptr.";
            return 100; // 返回-1作为错误代码
        }

        double total_distance = 0.0;
        int count = 0; // 计算有效配对的点数
        for (const auto &src_point : source_->points)
        {
            Vec3d q = ToVec3d(src_point);
            q = sim3_s_ * result.so3().matrix() * q + result.translation(); // 转换之后的q
            //q = result * q;                                          // 转换之后的q
            std::vector<int> nn_indices;                             // 用于存储最近邻的索引
            kdtree_->GetClosestPoint(ToPointType(q), nn_indices, 1); // 获取最近的一个点

            if (!nn_indices.empty())
            {
                auto closest_point = ToVec3d(target_->points[nn_indices[0]]);
                double distance = (q - closest_point).norm();

                // if (distance <= options_.max_nn_distance_)
                if (distance <= 10)
                { // 只计算距离小于某个阈值的点对
                    total_distance += distance;
                    count++;
                }
            }
        }

        if (count == 0)
            return 100;                // 如果没有有效的点对，返回错误代码
        return total_distance / count; // 返回平均距离作为适应性评分
    }

} // namespace aiper_relocalization_ns