/************************************************************
 *

 * // created by: zhangcheng

 *
 ***********************************************************/

#include "hj/odom.h"

void controlC(int sig) {

  hj::OdomNode::abort();

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "dlo_odom_node");
  ros::NodeHandle nh("~");

  signal(SIGTERM, controlC);
  sleep(0.5);

  hj::OdomNode node(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
