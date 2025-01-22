/*
 * @Description: 订阅imu数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:17
 */
#ifndef LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "localization/common/imu.h"
using namespace sad;
namespace localization {
class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMU>& deque_imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMU> new_imu_data_; 
};
}
#endif