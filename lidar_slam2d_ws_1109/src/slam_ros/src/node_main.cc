#include <ros/ros.h>
#include "node.h"

int main(int argc, char** argv) {

  ::ros::init(argc, argv, "slam_2d_node");
  ::ros::start();

  HJ_Slam2d_node node;

  ros::spin();

  ::ros::shutdown();
}