/*
 * @Description: 订阅uwb数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 16:44:18
 */
#include "localization/subscriber/uwb_subscriber.hpp"

#include "glog/logging.h"

namespace localization{
UWBSubscriber::UWBSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
        std::cout<<"topic_name:"<<topic_name<<std::endl;
    subscriber_ = nh_.subscribe(topic_name, buff_size, &UWBSubscriber::msg_callback, this);
}

void UWBSubscriber::msg_callback(const cf_msgs::Tdoa::ConstPtr& msg) {
    UWB uwb_data;
    uwb_data.timestamp_ = msg->header.stamp.toSec();
    uwb_data.idA_ = msg->idA;
    uwb_data.idB_ = msg->idB;
    uwb_data.data_ = msg->data;
    // std::cout<<"uwb_data.timestamp_:"<<std::to_string(uwb_data.timestamp_)<<std::endl;
    // std::cout<<"uwb_data.idA_:"<<uwb_data.idA_<<std::endl;
    // std::cout<<"uwb_data.idB_:"<<uwb_data.idB_<<std::endl;
    // std::cout<<"uwb_data.data_:"<<uwb_data.data_<<std::endl;
    new_uwb_data_.push_back(uwb_data);
}

void UWBSubscriber::ParseData(std::deque<UWB>& uwb_data_buff) {
    if (new_uwb_data_.size() > 0) {
        uwb_data_buff.insert(uwb_data_buff.end(), new_uwb_data_.begin(), new_uwb_data_.end());
        new_uwb_data_.clear();
    }
}
}