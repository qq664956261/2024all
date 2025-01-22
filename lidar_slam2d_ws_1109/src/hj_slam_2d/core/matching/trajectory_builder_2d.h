#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include "../../util/point_cloud.h"
#include "../undistortion/pose_extrapolator.h"
#include "real_time_correlative_scan_matcher_2d.h"
#include "../../util/grid_map/submap_2d.h"
#include "../../util/trajectory_node.h"
#include "../../util/motion_filter.h"

//后面优化
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace hjSlam_2d
{
    namespace mapping
    {
        class TrajectoryBuilder2D
        {
        public:
            struct InsertionResult
            {
                std::shared_ptr<const TrajectoryNode::Data> constant_data;
                std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
            };

            struct MatchingResult
            {
                common::Time time;
                transform::Rigid3d local_pose;
                hjSlam_2d::RangeData range_data_in_local;
                // 'nullptr' if dropped by the motion filter.
                std::unique_ptr<const InsertionResult> insertion_result;
            };

        public:
            explicit TrajectoryBuilder2D();
            ~TrajectoryBuilder2D();

            TrajectoryBuilder2D(const TrajectoryBuilder2D &) = delete;
            TrajectoryBuilder2D &operator=(const TrajectoryBuilder2D &) = delete;

            std::unique_ptr<MatchingResult>
             addRangeData(const hjSlam_2d::PointCloudWithIntensities &range_data);
            void initializeExtrapolator(const common::Time time);

            void AddOdometryData(const sensor::OdometryData &odometry_data);
            void addImuData(const sensor::ImuData& imu_data);

            // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
            // observed pose, or nullptr on failure.
            std::unique_ptr<transform::Rigid2d> ScanMatch(
                common::Time time, const transform::Rigid2d &pose_prediction,
                const hjSlam_2d::PointCloud &point_cloud);

            std::unique_ptr<InsertionResult> InsertIntoSubmap(
                common::Time time, const hjSlam_2d::RangeData &range_data_in_local,
                const hjSlam_2d::PointCloud &filtered_gravity_aligned_point_cloud,
                const transform::Rigid3d &pose_estimate,
                const Eigen::Quaterniond &gravity_alignment);

            void setLocal_slam_result_callback(std::function<void(int /* trajectory ID */, common::Time,
                                   transform::Rigid3d /* local pose estimate */,
                                   hjSlam_2d::RangeData /* in local frame */)> local_slam_result_callback)
                                   {local_slam_result_callback_ = local_slam_result_callback;};

            void addPCLCloud(const hjSlam_2d::PointCloud &pts);
            void setMapPointToMatched(const hjSlam_2d::PointCloud &pts);

        private:
            std::function<void(int /* trajectory ID */, common::Time,
                                   transform::Rigid3d /* local pose estimate */,
                                   hjSlam_2d::RangeData /* in local frame */) > local_slam_result_callback_;

            ActiveSubmaps2D active_submaps_;
            std::unique_ptr<PoseExtrapolator> extrapolator_;
            hjSlam_2d::RangeData accumulated_range_data_;
            scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
            MotionFilter motion_filter_;
            // scan_matching::CeresScanMatcher2D ceres_scan_matcher_;//替换ICP

            //std::deque<hjSlam_2d::PointCloud> range_data_toMatched;
            std::deque<pcl::PointCloud<pcl::PointXYZ>> points_to_matched_;
            hjSlam_2d::PointCloud mapObstructPoint;

            bool use_imu = false;
            time_t begin_time, end_time;
            double cost_time;

        };

    } // namespace mapping

} // namespace hjSlam_2d

#endif // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_