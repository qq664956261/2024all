/*
 * @Description: odometry 信息发布
 * @Author: zhang cheng
 * @Date: 2024-04-11 21:05:47
 */
#ifndef LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

namespace localization {
class OdometryPublisher {
  public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Publisher path_pub_;
    nav_msgs::Odometry odometry_;
    nav_msgs::Path path_;
};
}
#endif