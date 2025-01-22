/*
 * @Description: 订阅uwb数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:17
 */
#ifndef LOCALIZATION_SUBSCRIBER_UWB_SUBSCRIBER_HPP_
#define LOCALIZATION_SUBSCRIBER_UWB_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "cf_msgs/Tdoa.h"
#include "localization/common/uwb.h"
using namespace sad;
namespace localization {
class UWBSubscriber {
  public:
    UWBSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    UWBSubscriber() = default;
    void ParseData(std::deque<UWB>& uwb_data_buff);

  private:
    void msg_callback(const cf_msgs::Tdoa::ConstPtr& msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<UWB> new_uwb_data_; 
};
}
#endif