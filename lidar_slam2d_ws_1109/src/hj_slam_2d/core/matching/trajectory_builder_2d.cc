#include "trajectory_builder_2d.h"
#include "../../util/rigid_transform.h"
#include "../../util/grid_map/submap_2d.h"
#include "../../util/time.h"
#include "../../util/voxel_filter.h"
#include "registration/icp_svd_registration.h"
#include "registration/icp_gauss_Newton.h"

#include "gicp/fast_gicp.hpp"
#include "gicp/fast_gicp_st.hpp"
#include "gicp/fast_gicp_impl.hpp"
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <ctime>

namespace hjSlam_2d
{
    namespace mapping
    {

        TrajectoryBuilder2D::TrajectoryBuilder2D()
        {
        }

        TrajectoryBuilder2D::~TrajectoryBuilder2D()
        {
        }

        /*
        @param: range_data雷达坐标系点云数据
        */
        std::unique_ptr<TrajectoryBuilder2D::MatchingResult>
        TrajectoryBuilder2D::addRangeData(const hjSlam_2d::PointCloudWithIntensities &range_data)
        {
            // 以最后一个点为这一帧点云的时间戳
            std::cout << "==================data come in=======================" << std::endl;
            // scan_time 每一帧点云最后一个点的时间戳
            const common::Time &scan_time = range_data.time;
            if (!use_imu)
                initializeExtrapolator(scan_time);
            if (extrapolator_ == nullptr)
            {
                // Until we've initialized the extrapolator with our first IMU message, we
                // cannot compute the orientation of the rangefinder.
                // LOG(INFO) << "Extrapolator not yet initialized.";
                std::cout << "Extrapolator not yet initialized." << std::endl;
                return nullptr;
            }

            begin_time = clock();
            // 使用位姿外推器，带求取的每个点的pose，默认在一帧数据的时间内做匀速运动
            std::vector<transform::Rigid3f> range_data_poses;
            range_data_poses.reserve(range_data.points.size());
            bool warned = false;
            for (const auto &range_point : range_data.points)
            {
                // std::cout << "=========range_point.time ========" << range_point.time << std::endl;
                // range_point.time  每个点的在一帧点云当中的相对时间间隔
                common::Time time_point = scan_time + common::FromSeconds(range_point.time); 
                // GetLastExtrapolatedTime() 上一帧数据的时间
                if (time_point < extrapolator_->GetLastExtrapolatedTime())                  
                {
                    if (!warned)
                    {
                        std::cout
                            << "Timestamp of individual range data point jumps backwards from "
                            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point << std::endl;
                        warned = true;
                    }
                    time_point = extrapolator_->GetLastExtrapolatedTime();
                }
                range_data_poses.push_back(extrapolator_->extrapolatePose(time_point).cast<float>()); // 计算一帧点云数据中每个点的位姿
            }

            // 去除畸变
            accumulated_range_data_ = hjSlam_2d::RangeData{{}, {}, {}, {}};
            for (size_t i = 0; i < range_data.points.size(); ++i)
            {
                hjSlam_2d::RangefinderPoint pt;
                // const Eigen::Vector3f hit_in_local = range_data_poses[i] * range_data.points[i].position; // 校畸变
                const Eigen::Vector3f hit_in_local = range_data_poses.back() * range_data.points[i].position; // 不校畸变
                // const Eigen::Vector3f delta = hit_in_local;
                const float range = range_data.points[i].position.head(2).norm();
                if (range >= 0.05 /*options_.min_range()*/) // TODO: 此处直接指定了数值，包括下面。
                {
                    if (range <= 15.0 /*options_.max_range()*/)
                    {
                        pt.position = hit_in_local;
                        accumulated_range_data_.returns.push_back(pt);
                    }
                    else
                    {
                        // 大于15时，设置为15，且属性为misses?
                        pt.position = /*options_.missing_data_ray_length()*/ 15.0 / range * hit_in_local;
                        accumulated_range_data_.misses.push_back(pt);
                    }
                }
            }
            // accumulated_range_data_ 现在目前在local坐标系
            end_time = clock();
            cost_time = double(end_time - begin_time) / 1000;
            std::cout << "===========畸变去除=============" << cost_time  << "ms" << std::endl;

            // 暂时没有使用
            accumulated_range_data_.origin = range_data_poses.back().translation();
            // 从local坐标系转lidar坐标系
            accumulated_range_data_.returns = TransformPointCloud(accumulated_range_data_.returns, range_data_poses.back().inverse());
            accumulated_range_data_.misses = TransformPointCloud(accumulated_range_data_.misses, range_data_poses.back().inverse());
            
            if (accumulated_range_data_.returns.empty()) 
                return nullptr;

            begin_time = clock();
            hjSlam_2d::RangeData filtered_accumulated_range_data = hjSlam_2d::RangeData{
                accumulated_range_data_.origin,
                sensor::VoxelFilter(accumulated_range_data_.returns, 0.025),
                sensor::VoxelFilter(accumulated_range_data_.misses, 0.025)};

            // accumulated_range_data_ = hjSlam_2d::RangeData{
            //     accumulated_range_data_.origin,
            //     sensor::VoxelFilter(accumulated_range_data_.returns, 0.025),
            //     sensor::VoxelFilter(accumulated_range_data_.misses, 0.025)};
            end_time = clock();
            std::cout << "===========点云滤波1=============" << double(end_time - begin_time) / 1000  << "ms" << std::endl;

            // const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
            // extrapolator_->EstimateGravityOrientation(scan_time));
            // Computes a gravity aligned pose prediction.(去除重力对齐)
            const transform::Rigid3d pose_prediction_3d = extrapolator_->extrapolatePose(scan_time);
            const transform::Rigid2d pose_prediction = transform::Project2D(pose_prediction_3d);

            // std::cout << "pose_prediction: " << pose_prediction.translation().x()<< " " << pose_prediction.translation().y() <<" "
            // << pose_prediction.rotation().angle() << std::endl;
            // 点云滤波处理
            std::cout << "======before filter returns.size:=========" << accumulated_range_data_.returns.size() << std::endl;
            begin_time = clock();
            const hjSlam_2d::PointCloud &filtered_point_cloud =
                sensor::AdaptiveVoxelFilter(filtered_accumulated_range_data.returns);
            // const hjSlam_2d::PointCloud &filtered_point_cloud =
            //     sensor::AdaptiveVoxelFilter(accumulated_range_data_.returns);
            end_time = clock();
            std::cout << "===========点云滤波2=============" << double(end_time - begin_time) / 1000  << "ms" << std::endl;
            std::cout << "===========match_points size:========" << filtered_point_cloud.size() << std::endl;

            if (filtered_point_cloud.empty())
                return nullptr;
            
            std::unique_ptr<transform::Rigid2d> pose_estimate_2d = ScanMatch(scan_time, pose_prediction, filtered_point_cloud);
            // std::cout << "pose_estimate_2d: " << pose_estimate_2d->translation().x() << " " << pose_estimate_2d->translation().y() << " "
            //           << pose_estimate_2d->rotation().angle() << std::endl;

            if (pose_estimate_2d == nullptr)
            {
                std::cout << "Scan matching failed." << std::endl;
                return nullptr;
            }

            const transform::Rigid3d pose_estimate_3d = transform::Embed3D(*pose_estimate_2d) /** gravity_alignment*/;
            extrapolator_->addPose(scan_time, pose_estimate_3d);

            hjSlam_2d::RangeData range_data_in_local = TransformRangeData(accumulated_range_data_,
                                                                           transform::Embed3D((*pose_estimate_2d).cast<float>()));

            std::unique_ptr<TrajectoryBuilder2D::InsertionResult> insertion_result = InsertIntoSubmap(
                scan_time, range_data_in_local, filtered_point_cloud,
                pose_estimate_3d, pose_estimate_3d.rotation() /*改动gravity_alignment.rotation()*/);

            const hjSlam_2d::PointCloud &filtered_point_cloud_in_local = TransformPointCloud(filtered_point_cloud,
                                                                                              transform::Embed3D(pose_estimate_2d->cast<float>()));

            // 增加targetCloud滤波
            hjSlam_2d::PointCloud newFilteredCLoud;
            for (auto scan_pt : filtered_point_cloud_in_local.points())
            {
                for (auto map_pt : mapObstructPoint.points())
                {
                    float x_bound_min = map_pt.position.x() - 0.025;
                    float x_bound_max = map_pt.position.x() + 0.025;
                    float y_bound_min = map_pt.position.y() - 0.025;
                    float y_bound_max = map_pt.position.y() + 0.025;
                    if (scan_pt.position.x() > x_bound_min && scan_pt.position.x() < x_bound_max &&
                        scan_pt.position.y() > y_bound_min && scan_pt.position.y() < y_bound_max)
                    {
                        newFilteredCLoud.push_back({Eigen::Vector3f(scan_pt.position.x(), scan_pt.position.y(), scan_pt.position.z())});
                        break;
                    }
                }
            }
            addPCLCloud(newFilteredCLoud);
            // addPCLCloud(filtered_point_cloud_in_local);

            for (auto it : points_to_matched_)
            {
                for (auto pt : it.points)
                {
                    range_data_in_local.points_to_matched.push_back({Eigen::Vector3f(pt.x, pt.y, pt.z)});
                }
            }

            if (local_slam_result_callback_)
            {
                local_slam_result_callback_(
                    0, scan_time, pose_estimate_3d,
                    std::move(range_data_in_local));
            }

            return std::make_unique<MatchingResult>(
                MatchingResult{scan_time, pose_estimate_3d, std::move(range_data_in_local),
                               std::move(insertion_result)});

        }

        std::unique_ptr<TrajectoryBuilder2D::InsertionResult> TrajectoryBuilder2D::InsertIntoSubmap(
            common::Time time, const hjSlam_2d::RangeData &range_data_in_local,
            const hjSlam_2d::PointCloud &filtered_gravity_aligned_point_cloud,
            const transform::Rigid3d &pose_estimate,
            const Eigen::Quaterniond &gravity_alignment)
        {
            // 距离太近或时间太近就不插入子图 后续采样
            if (motion_filter_.IsSimilar(time, pose_estimate))
            {
                return nullptr;
            }

            std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
                active_submaps_.InsertRangeData(range_data_in_local);

            return std::make_unique<InsertionResult>(InsertionResult{
                std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
                    time,
                    gravity_alignment,
                    filtered_gravity_aligned_point_cloud,
                    {}, // 'high_resolution_point_cloud' is only used in 3D.
                    {}, // 'low_resolution_point_cloud' is only used in 3D.
                    {}, // 'rotational_scan_matcher_histogram' is only used in 3D.
                    pose_estimate}),
                std::move(insertion_submaps)});
        }

        std::unique_ptr<transform::Rigid2d> TrajectoryBuilder2D::ScanMatch(
            const common::Time time, const transform::Rigid2d &pose_prediction,
            const hjSlam_2d::PointCloud &match_points)
        {
            if (active_submaps_.submaps().empty())
                return std::make_unique<transform::Rigid2d>(pose_prediction);
            
            static int allcount = 0;
            static int validcount = 0;
            auto pose_observation = std::make_unique<transform::Rigid2d>();

            // ************************************CSM*************************************************
            std::shared_ptr<const Submap2D> matching_submap = active_submaps_.submaps().front();
            transform::Rigid2d csm_pose = pose_prediction;
            begin_time = clock();
            const double csm_score = real_time_correlative_scan_matcher_.Match(
                pose_prediction, match_points,
                *matching_submap->grid(), &csm_pose);
            end_time = clock();
            allcount++;
            cost_time = double(end_time - begin_time) / 1000;
            *pose_observation = csm_pose;
            std::cout << "==========csm_runtime:==========" << cost_time << "ms" << std::endl;
            std::cout << "==========csm_score:============" << csm_score << std::endl;
            //****************************************************************************************

            if (csm_score < 1 && points_to_matched_.size() > 1)
            {
                // 直方图
                // kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
                // const hjSlam_2d::PointCloud cur_points_in_local = TransformPointCloud(match_points,
                //                                                                        transform::Embed3D(initial_ceres_pose.cast<float>()));
                // 筛选5m之内的点
                //  hjSlam_2d::PointCloud sourcePts;
                //  for (auto it : match_points.points())
                //  {
                //      if (abs(it.position.x()) < 6.5 && abs(it.position.y()) < 6.5)
                //      {
                //          hjSlam_2d::RangefinderPoint pt{Eigen::Vector3f(it.position.x(), it.position.y(), it.position.z())};
                //          sourcePts.push_back(pt);
                //      }
                //  }

                //************************************ICP********************************************
                // std::cout << "cur_points_in_local.size():" << mapPoint_toMatched.size() << std::endl;
                // pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
                // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
                // pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
                const hjSlam_2d::PointCloud cur_points_in_local = TransformPointCloud(match_points,
                                                                                       transform::Embed3D(csm_pose.cast<float>()));

                // for (auto it : cur_points_in_local.points())
                // {
                //     sourceCloud->points.push_back(pcl::PointXYZ(it.position.x(), it.position.y(), it.position.z()));
                // }

                hjSlam_2d::PointCloud scans_toMatched;
                for (auto it : points_to_matched_)
                {
                    // *targetCloud += it;
                    for (auto pt : it.points)
                    {
                        scans_toMatched.push_back({Eigen::Vector3f(pt.x, pt.y, pt.z)});
                    }
                }

                ICPSVDRegistration svd_icp;
                Eigen::Matrix3f csm_pose_mat = transform::rigid2ToEigen3(csm_pose.cast<float>());
                begin_time = clock();
                svd_icp.setInputTarget(scans_toMatched);
                svd_icp.setRegistrationParam(0.05, 0.001, 0.001, 50);
                svd_icp.scanMatch(csm_pose_mat, match_points, *matching_submap->grid(), csm_score);
                end_time = clock();
                cost_time = double(end_time - begin_time) / 1000;
                std::cout << "============svd_icp_runtime:=============" << cost_time << "ms" << std::endl;

                // ICPRegistrationManual guss_icp;
                // begin1 = clock();
                // guss_icp.SetRegistrationParam(0.01, 50);
                // guss_icp.setInputTarget(scans_toMatched);
                // guss_icp.scanMatch(cur_points_in_local);
                // end1 = clock();
                // ret = double(end1 - begin1) / 1000;
                // std::cout << "===========guss_icp_runtime:=============" << ret << "ms" << std::endl;

                //=====================test_kdtree=============================
                // pcl::PointXYZ pSearch = sourceCloud->points[0];
                // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                // kdtree.setInputCloud(targetCloud);
                // std::vector<int> ptIdxByKNN(10);                       // 存储查询点近邻索引
                // std::vector<float> ptKNN(10);                               // 存储近邻点对应距离平方
                // kdtree.nearestKSearch(pSearch, 10, ptIdxByKNN, ptKNN); // 执行K近邻搜索
                // for (auto it : ptIdxByKNN)
                //     std::cout << "icp_knn_indice:" << it << std::endl;
                // for (auto it : ptKNN)
                //     std::cout << "icp_knn_dis:" << it << std::endl;
                //===========================================================
                // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                // begin1 = clock();
                // icp.setInputTarget(targetCloud); // targetCloud
                // icp.setMaxCorrespondenceDistance(0.05);
                // icp.setMaximumIterations(1);
                // icp.setTransformationEpsilon(1e-8);
                // icp.setEuclideanFitnessEpsilon(0.01);
                // icp.setInputSource(sourceCloud); // sourceCloud
                // icp.align(*finalCloud);
                // end1 = clock();
                // ret = double(end1 - begin1) / 1000;
                // // std::cout << "===========pcl_kdtree_runtime:=============" << ret << "ms" << std::endl;
                // std::cout << "===========pcl_icp_runtime:=============" << ret << "ms" << std::endl;

                // fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fast_gicp;
                // fast_gicp.setMaxCorrespondenceDistance(0.05);
                // // MIN_EIG;FROBENIUS;NONE;NORMALIZED_MIN_EIG;PLANE
                // fast_gicp.setRegularizationMethod(fast_gicp::RegularizationMethod::NORMALIZED_MIN_EIG);
                // fast_gicp.setRANSACOutlierRejectionThreshold(0.05);
                // fast_gicp.setTransformationEpsilon(0.0005);
                // fast_gicp.setMaximumIterations(50);
                // fast_gicp.setInputSource(sourceCloud); // sourceCloud
                // fast_gicp.setInputTarget(targetCloud); // targetCloud
                // fast_gicp.align(*finalCloud);

                // Eigen::Matrix4d transform = icp.getFinalTransformation().cast<double>();
                // // Eigen::Matrix3d rotation_matrix = transform.block(0, 0, 3, 3);
                // // Eigen::Vector3d trans = transform.block(0, 3, 3, 1);
                // // Eigen::Quaterniond quaternion2(rotation_matrix);
                // // Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2, 1, 0);
                // Eigen::Quaterniond qr(transform.block<3, 3>(0, 0));
                // qr.normalize();
                // Eigen::Vector3d trans = transform.block<3, 1>(0, 3);
                // Eigen::Vector3d eulerAngle = qr.matrix().eulerAngles(0, 1, 2);

                Eigen::Matrix3d transform = svd_icp.getFinalTransformation().cast<double>();
                double angle = std::atan2(transform(1, 0), transform(0, 0));
                Eigen::Vector2d trans = transform.block<2, 1>(0, 2);

                // std::cout << "diff angle" << diff_angle << std::endl;
                // std::cout << "diff trans" << diff_trans << std::endl;
                //  std::cout << "=====eulerAngle=====" << angle << std::endl;
                //  std::cout << "=======trans=====" << trans << std::endl;
                // *pose_observation = initial_ceres_pose;
                // Eigen::Matrix3d csm_pose_mat = transform::rigid2ToEigen3(csm_pose);
                // Eigen::Matrix3d final_pose_mat = transform * csm_pose_mat;
                // double angle = std::atan2(final_pose_mat(1, 0), final_pose_mat(0, 0));
                // Eigen::Vector2d trans = final_pose_mat.block<2, 1>(0, 2);
                // std::cout << "Angle:" << angle - csm_pose.rotation().angle() << std::endl;
                // std::cout << "Trans:" << trans.x() - csm_pose.translation().x() << " "<<   trans.y() - csm_pose.translation().y()<< std::endl;
                //  std::cout << "=====eulerAngle=====" << angle << std::endl;
                //  std::cout << "=======trans=====" << trans << std::endl;
                // transform::Rigid2d icp_pose = transform::Rigid2d(
                //     {csm_pose.translation().x() + trans.x(),
                //      csm_pose.translation().y() + trans.y()},
                //     csm_pose.rotation() * Eigen::Rotation2Dd(angle)); // angle  eulerAngle.z()

                transform::Rigid2d icp_pose = transform::Rigid2d({trans.x(), trans.y()}, Eigen::Rotation2Dd(angle));
                const hjSlam_2d::PointCloud cur_points_icp = TransformPointCloud(match_points,
                                                                                  transform::Embed3D(icp_pose.cast<float>()));

                // hjSlam_2d::PointCloud cur_points_icp;
                // for (int i = 0; i < cur_points_in_local.size(); ++i)
                // {
                //     float new_x = cur_points_in_local[i].position.x() * transform(0, 0) + cur_points_in_local[i].position.y() * transform(0, 1) +  transform(0, 2);
                //     float new_y = cur_points_in_local[i].position.x() * transform(1, 0) + cur_points_in_local[i].position.y() * transform(1, 1) +  transform(1, 2);
                //     cur_points_icp.push_back({Eigen::Vector3f(new_x, new_y, 0)});
                // }

                const double icp_score = real_time_correlative_scan_matcher_.compute_occupied_score(cur_points_icp,
                                                                                                    *matching_submap->grid());
                std::cout << "===========final_icp_score:=============" << icp_score << std::endl;

                if (icp_score > csm_score)
                {
                    *pose_observation = icp_pose;
                    validcount++;
                    std::cout << "==============use ICP================" << std::endl;
                }
                
            }

            std::cout << "allcount:" << allcount << std::endl;
            std::cout << "validcount:" << validcount << std::endl;

            /*
            //替换ICP
                ceres::Solver::Summary summary;
                ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                                          filtered_gravity_aligned_point_cloud,
                                          *matching_submap->grid(), pose_observation.get(),
                                          &summary);
                if (pose_observation)
                {
                    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
                    const double residual_distance =
                        (pose_observation->translation() - pose_prediction.translation())
                            .norm();
                    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
                    const double residual_angle =
                        std::abs(pose_observation->rotation().angle() -
                                 pose_prediction.rotation().angle());
                    kScanMatcherResidualAngleMetric->Observe(residual_angle);
                } */

            return pose_observation;
        }

        void TrajectoryBuilder2D::initializeExtrapolator(const common::Time time)
        {
            if (extrapolator_ != nullptr)
            {
                return;
            }
            // We derive velocities from poses which are at least 1 ms apart for numerical
            // stability. Usually poses known to the extrapolator will be further apart
            // in time and thus the last two are used.
            constexpr double extrapolation_estimation_time_sec = 0.001;
            // TODO(gaschler): Consider using InitializeWithImu as 3D does.
            extrapolator_ = std::make_unique<PoseExtrapolator>(
                ::hjSlam_2d::common::FromSeconds(extrapolation_estimation_time_sec), 9.8
                /*options_.imu_gravity_time_constant()*/);
            extrapolator_->addPose(time, transform::Rigid3d::Identity());
        }

        void TrajectoryBuilder2D::AddOdometryData(
            const sensor::OdometryData &odometry_data)
        {
            if (extrapolator_ == nullptr)
            {
                // Until we've initialized the extrapolator we cannot add odometry data.
                std::cout << "Extrapolator not yet initialized." << std::endl;
                return;
            }
            extrapolator_->AddOdometryData(odometry_data);
        }

        void TrajectoryBuilder2D::addImuData(const sensor::ImuData &imu_data)
        {
            // CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
            initializeExtrapolator(imu_data.time);
            extrapolator_->addImuData(imu_data);
        }

        void TrajectoryBuilder2D::addPCLCloud(const hjSlam_2d::PointCloud &pts)
        {
            pcl::PointCloud<pcl::PointXYZ> points;
            for (auto it : pts.points())
            {
                points.points.push_back(pcl::PointXYZ(it.position.x(), it.position.y(), it.position.z()));
            }
            points_to_matched_.push_back(points);

            if (points_to_matched_.size() > 90)
                points_to_matched_.pop_front();
        }

        void TrajectoryBuilder2D::setMapPointToMatched(const hjSlam_2d::PointCloud &pts)
        {
            mapObstructPoint = pts;
        }

    } // namespace mapping

} // namespace hjSlam_2d