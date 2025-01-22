#include "slam_catkin.h"
#include <chrono>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh("~");


  ROS_INFO("\033[1;32m---->\033[0m mapper Started.");
  ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
  spinner.spin();

  printf("ROS to shutdown.\n");
  ros::shutdown();
  return 0;
}
