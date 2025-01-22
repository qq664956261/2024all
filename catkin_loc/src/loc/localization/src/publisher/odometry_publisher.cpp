/*
 * @Description: 里程计信息发布
 * @Author: zhang cheng
 * @Date: 2020-04-11 21:11:44
 */
#include "localization/publisher/odometry_publisher.hpp"

namespace localization
{
    OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         int buff_size)
        : nh_(nh)
    {

        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", buff_size);
        path_.header.frame_id = base_frame_id;
    }

    void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix)
    {
        odometry_.header.stamp = ros::Time::now();

        // set the position
        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();

        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = transform_matrix(0, 3);
        this_pose_stamped.pose.position.y = transform_matrix(1, 3);
        this_pose_stamped.pose.position.z = transform_matrix(2, 3);
        this_pose_stamped.header.frame_id = path_.header.frame_id;
        path_.poses.push_back(this_pose_stamped);

        path_pub_.publish(path_);

        publisher_.publish(odometry_);
    }
}