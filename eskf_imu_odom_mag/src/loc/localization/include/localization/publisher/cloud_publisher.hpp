/*
 * @Description: 在ros中发布点云
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:30
 */

#ifndef LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "localization/sensor_data/cloud_data.hpp"

namespace localization {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif