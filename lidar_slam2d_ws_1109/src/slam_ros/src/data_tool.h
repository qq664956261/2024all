#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H

#include "../../hj_slam_2d/util/point_cloud.h"
#include "../../hj_slam_2d/util/time.h"
#include "../../hj_slam_2d/util/rigid_transform.h"
#include "../../hj_slam_2d/util/sensor_data.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <tuple>

namespace slam2dTool_ros
{
 
    ::ros::Time ToRos(hjSlam_2d::common::Time time);

    hjSlam_2d::common::Time FromRos(const ::ros::Time &time);

    hjSlam_2d::PointCloudWithIntensities
    ToPointCloudWithIntensities(const sensor_msgs::LaserScan &msg);

    std::unique_ptr<hjSlam_2d::sensor::OdometryData> ToOdometryData(
        const nav_msgs::Odometry::ConstPtr &msg);

    std::unique_ptr<hjSlam_2d::sensor::ImuData> ToImuData(
        const sensor_msgs::Imu::ConstPtr &msg);

    hjSlam_2d::transform::Rigid3d ToRigid3d(
        const geometry_msgs::TransformStamped &transform);

    ::hjSlam_2d::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose &pose);

    Eigen::Vector3d ToEigen(const geometry_msgs::Vector3 &vector3);

    Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion &quaternion);

    // Converts from WGS84 (latitude, longitude, altitude) to ECEF.
    Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                     double altitude);

}

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H