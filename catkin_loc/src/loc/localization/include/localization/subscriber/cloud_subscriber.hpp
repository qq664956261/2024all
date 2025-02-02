/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:30
 */

#ifndef LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "localization/sensor_data/cloud_data.hpp"

namespace localization {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<CloudData> new_cloud_data_;
};
}

#endif