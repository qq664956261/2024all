/************************************************************
// created by: zhangcheng
 ***********************************************************/

#include "hj/dlo.h"

class hj::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  static void abort() {
    abort_ = true;
  }

  void start();
  void stop();

private:

  void abortTimerCB(const ros::TimerEvent& e);
  void publishTimerCB(const ros::TimerEvent& e);

  void keyframeCB(const sensor_msgs::PointCloud2ConstPtr& keyframe);

  bool savePcd(hj_slam::save_pcd::Request& req,
               hj_slam::save_pcd::Response& res);

  void getParams();

  ros::NodeHandle nh;
  ros::Timer abort_timer;
  ros::Timer publish_timer;

  ros::Subscriber keyframe_sub;
  ros::Publisher map_pub;

  ros::ServiceServer save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlo_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  ros::Time map_stamp;
  std::string odom_frame;

  bool publish_full_map_;
  double publish_freq_;
  double leaf_size_;

  static std::atomic<bool> abort_;

};
