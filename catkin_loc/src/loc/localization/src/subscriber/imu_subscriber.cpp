/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "localization/subscriber/imu_subscriber.hpp"

#include "glog/logging.h"

namespace localization{
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMU imu_data;
    imu_data.timestamp_ = imu_msg_ptr->header.stamp.toSec();

    imu_data.acce_[0] = imu_msg_ptr->linear_acceleration.x * 9.81;
    imu_data.acce_[1] = imu_msg_ptr->linear_acceleration.y * 9.81;
    imu_data.acce_[2] = imu_msg_ptr->linear_acceleration.z * 9.81;

    imu_data.gyro_[0] = imu_msg_ptr->angular_velocity.x * M_PI / 180.0;
    imu_data.gyro_[1] = imu_msg_ptr->angular_velocity.y * M_PI / 180.0;
    imu_data.gyro_[2] = imu_msg_ptr->angular_velocity.z * M_PI / 180.0;

    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;
    // std::cout<<"imu_data.timestamp_:"<<std::to_string(imu_data.timestamp_)<<std::endl;
    // std::cout<<"imu_data.acce_[0]:"<<imu_data.acce_[0]<<std::endl;
    // std::cout<<"imu_data.acce_[1]:"<<imu_data.acce_[1]<<std::endl;
    // std::cout<<"imu_data.acce_[2]:"<<imu_data.acce_[2]<<std::endl;

    new_imu_data_.push_back(imu_data);
}

void IMUSubscriber::ParseData(std::deque<IMU>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}
}