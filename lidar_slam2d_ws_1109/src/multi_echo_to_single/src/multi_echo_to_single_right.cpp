#include <ros/ros.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher scan_pub;

void callback(const sensor_msgs::MultiEchoLaserScan::ConstPtr& mul_scan) {
  sensor_msgs::LaserScan scan;
  scan.header = mul_scan->header;
  scan.angle_min = mul_scan->angle_min;
  scan.angle_max = mul_scan->angle_max;
  scan.angle_increment = mul_scan->angle_increment;
  scan.time_increment = mul_scan->time_increment;
  scan.scan_time = mul_scan->scan_time;
  scan.range_min = mul_scan->range_min;
  scan.range_max = mul_scan->range_max;

  for (int i = 0; i < mul_scan->ranges.size(); ++i) {
    scan.ranges.push_back(mul_scan->ranges[i].echoes[0]);  // Using the first echo
  }

  // Publish the converted LaserScan message
  scan_pub.publish(scan);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_echo_to_laser_scan");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/multi_echo_scan", 1000, callback);
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1000);

  ros::spin();
  return 0;
}
