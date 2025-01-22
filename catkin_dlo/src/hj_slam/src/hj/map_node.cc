/************************************************************
 *

 * // created by: zhangcheng

 *
 ***********************************************************/

#include "hj/map.h"

void controlC(int sig) {

  hj::MapNode::abort();

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "dlo_map_node");
  ros::NodeHandle nh("~");

  signal(SIGTERM, controlC);
  sleep(0.5);

  hj::MapNode node(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
