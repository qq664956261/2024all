#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber multi_echo_sub;
ros::Publisher single_echo_pub;

void convertMultiEchoToSingleLaserScan(const sensor_msgs::MultiEchoLaserScan& multi_echo_msg, sensor_msgs::LaserScan & single_echo_msg){
    single_echo_msg.header = multi_echo_msg.header;
    single_echo_msg.angle_max = multi_echo_msg.angle_max;
    single_echo_msg.angle_min = multi_echo_msg.angle_min;
    single_echo_msg.angle_increment = multi_echo_msg.angle_increment;
    single_echo_msg.time_increment = multi_echo_msg.time_increment;
    single_echo_msg.scan_time = multi_echo_msg.scan_time;
    single_echo_msg.range_max = multi_echo_msg.range_max;
    single_echo_msg.range_min = multi_echo_msg.range_min;

    // 转换数据
    for (int i = 0; i < multi_echo_msg.ranges.size(); ++i){
        single_echo_msg.ranges.push_back(multi_echo_msg.ranges[i].echoes[0]);
    }
}

void multi_echo_callback(const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg){
    ;
    sensor_msgs::LaserScan single_echo_msg;
    convertMultiEchoToSingleLaserScan(*msg, single_echo_msg);
    single_echo_pub.publish(single_echo_msg);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "multi_echo_to_single");
    ros::NodeHandle nh;
    multi_echo_sub = nh.subscribe("/horizontal_laser_2d", 10, multi_echo_callback);
    single_echo_pub = nh.advertise<sensor_msgs::LaserScan>("/horizontal_laser_2d_single_echo", 10);
    ros::spin();
    return 0;
}