#include "data_tool.h"
#include "Eigen/Geometry"

namespace slam2dTool_ros
{
    namespace
    {
        using ::hjSlam_2d::transform::Rigid3d;

        template <typename LaserMessageType>
        hjSlam_2d::PointCloudWithIntensities LaserScanToPointCloudWithIntensities(const LaserMessageType &msg)
        {

            hjSlam_2d::PointCloudWithIntensities point_cloud;
            float angle = msg.angle_min;

            /************Adjust timestamp for point frequency *********/
            // static float timestamp_current = 0;
            // static float timestamp_last = 0;
            // float time_increment_vary;
            // timestamp_last = timestamp_current;
            // timestamp_current = msg.header.stamp.toSec();
            // if (timestamp_last != 0)
            // {
            //     time_increment_vary = (timestamp_current - timestamp_last) / (6.2831852 / msg.angle_increment);
            // }
            // else
            //     time_increment_vary = msg.time_increment;
            /********************** end **************************/

            for (size_t i = 0; i < msg.ranges.size(); ++i)
            {
                const auto &first_echo = msg.ranges[i];
                if (msg.range_min <= first_echo && first_echo <= msg.range_max)
                {
                    const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());

                    const hjSlam_2d::TimedRangefinderPoint point{
                        rotation * (first_echo * Eigen::Vector3f::UnitX()),
                        i * msg.time_increment};
                    //*******************修改***********************************
                    // const hjSlam_2d::TimedRangefinderPoint point{
                    //     rotation * (first_echo * Eigen::Vector3f::UnitX()),
                    //     i * time_increment_vary};
                    //**********************************************************
                    point_cloud.points.push_back(point);
                }

                angle += msg.angle_increment;
            }

            // 对于顺时针雷达使用以下部分代码
            /*********** Adjust timestamp for point reverse **********/
            // std::vector<float> time_tempx;
            // std::vector<float> time_tempy;
            // for (auto &point : point_cloud.points)
            // {
            //     time_tempx.push_back(point.position.x());
            //     time_tempy.push_back(point.position.y());
            // }
            // int i_time = 0;
            // for (auto &point : point_cloud.points)
            // {
            //     point.position.x() = time_tempx[point_cloud.points.size() - 1 - i_time];
            //     point.position.y() = time_tempy[point_cloud.points.size() - 1 - i_time];
            //     i_time++;
            // }
            /*********************** end **********************/

            // 将一帧点云转为以最后一个点为起始点
            hjSlam_2d::common::Time timestamp = FromRos(msg.header.stamp);
            if (!point_cloud.points.empty())
            {
                const double duration = point_cloud.points.back().time;
                timestamp += hjSlam_2d::common::FromSeconds(duration);
                for (auto &point : point_cloud.points)
                {
                    point.time -= duration;
                }
            }

            point_cloud.time = timestamp;
            return point_cloud;
        }
    } // namespace

    ::ros::Time ToRos(::hjSlam_2d::common::Time time)
    {
        int64_t uts_timestamp = ::hjSlam_2d::common::ToUniversal(time);
        int64_t ns_since_unix_epoch =
            (uts_timestamp -
             ::hjSlam_2d::common::kUtsEpochOffsetFromUnixEpochInSeconds *
                 10000000ll) *
            100ll;
        ::ros::Time ros_time;
        ros_time.fromNSec(ns_since_unix_epoch);
        return ros_time;
    }

    // TODO(pedrofernandez): Write test.
    ::hjSlam_2d::common::Time FromRos(const ::ros::Time &time)
    {
        // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
        // exactly 719162 days before the Unix epoch.
        return ::hjSlam_2d::common::FromUniversal(
            (time.sec +
             ::hjSlam_2d::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                10000000ll +
            (time.nsec + 50) / 100); // + 50 to get the rounding correct.
    }

    hjSlam_2d::PointCloudWithIntensities ToPointCloudWithIntensities(const sensor_msgs::LaserScan &msg)
    {
        return LaserScanToPointCloudWithIntensities(msg);
    }


    std::unique_ptr<hjSlam_2d::sensor::OdometryData> ToOdometryData(
        const nav_msgs::Odometry::ConstPtr &msg)
    {
        const hjSlam_2d::common::Time time = FromRos(msg->header.stamp);
        return std::make_unique<hjSlam_2d::sensor::OdometryData>(
            hjSlam_2d::sensor::OdometryData{
                time, ToRigid3d(msg->pose.pose)/* sensor_to_tracking->inverse()*/});
    }

    std::unique_ptr<hjSlam_2d::sensor::ImuData> ToImuData(
        const sensor_msgs::Imu::ConstPtr &msg)
    {
        //               CHECK_NE(msg->linear_acceleration_covariance[0], -1)
        //       << "Your IMU data claims to not contain linear acceleration measurements "
        //          "by setting linear_acceleration_covariance[0] to -1. Cartographer "
        //          "requires this data to work. See "
        //          "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
        //   CHECK_NE(msg->angular_velocity_covariance[0], -1)
        //       << "Your IMU data claims to not contain angular velocity measurements "
        //          "by setting angular_velocity_covariance[0] to -1. Cartographer "
        //          "requires this data to work. See "
        //          "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

        const hjSlam_2d::common::Time time = FromRos(msg->header.stamp);

        //   const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
        //       time, CheckNoLeadingSlash(msg->header.frame_id));
        //   if (sensor_to_tracking == nullptr) {
        //     return nullptr;
        //   }
        //   CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        //       << "The IMU frame must be colocated with the tracking frame. "
        //          "Transforming linear acceleration into the tracking frame will "
        //          "otherwise be imprecise.";

        return std::make_unique<hjSlam_2d::sensor::ImuData>(hjSlam_2d::sensor::ImuData{
            time, /*sensor_to_tracking->rotation() * */ ToEigen(msg->linear_acceleration),
            /*sensor_to_tracking->rotation() * */ ToEigen(msg->angular_velocity)});
    }

    ::hjSlam_2d::transform::Rigid3d ToRigid3d(
        const geometry_msgs::TransformStamped &transform)
    {
        return Rigid3d(ToEigen(transform.transform.translation),
                       ToEigen(transform.transform.rotation));
    }

    ::hjSlam_2d::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose &pose)
    {
        return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                       ToEigen(pose.orientation));
    }

    Eigen::Vector3d ToEigen(const geometry_msgs::Vector3 &vector3)
    {
        return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }

    Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion &quaternion)
    {
        return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                                  quaternion.z);
    }

}