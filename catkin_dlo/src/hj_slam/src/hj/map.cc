/************************************************************
 *

 * // created by: zhangcheng

 *
 ***********************************************************/

#include "hj/map.h"

std::atomic<bool> hj::MapNode::abort_(false);

/**
 * Constructor
 **/

hj::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle)
{

  this->getParams();

  this->abort_timer = this->nh.createTimer(ros::Duration(0.01), &hj::MapNode::abortTimerCB, this);

  if (this->publish_full_map_)
  {
    this->publish_timer = this->nh.createTimer(ros::Duration(this->publish_freq_), &hj::MapNode::publishTimerCB, this);
  }

  this->keyframe_sub = this->nh.subscribe("keyframes", 1, &hj::MapNode::keyframeCB, this);
  this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 1);

  this->save_pcd_srv = this->nh.advertiseService("save_pcd", &hj::MapNode::savePcd, this);

  // initialize map
  this->dlo_map = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  ROS_INFO("DLO Map Node Initialized");
}

/**
 * Destructor
 **/

hj::MapNode::~MapNode() {}

/**
 * Get Params
 **/

void hj::MapNode::getParams()
{

  ros::param::param<std::string>("~dlo/odomNode/odom_frame", this->odom_frame, "odom");
  ros::param::param<bool>("~dlo/mapNode/publishFullMap", this->publish_full_map_, true);
  ros::param::param<double>("~dlo/mapNode/publishFreq", this->publish_freq_, 1.0);
  ros::param::param<double>("~dlo/mapNode/leafSize", this->leaf_size_, 0.5);

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0, 1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;
}

/**
 * Start Map Node
 **/

void hj::MapNode::start()
{
  ROS_INFO("Starting DLO Map Node");
}

/**
 * Stop Map Node
 **/

void hj::MapNode::stop()
{
  ROS_WARN("Stopping DLO Map Node");

  // shutdown
  ros::shutdown();
}

/**
 * Abort Timer Callback
 **/

void hj::MapNode::abortTimerCB(const ros::TimerEvent &e)
{
  if (abort_)
  {
    stop();
  }
}

/**
 * Publish Timer Callback
 **/

void hj::MapNode::publishTimerCB(const ros::TimerEvent &e)
{

  if (this->dlo_map->points.size() == this->dlo_map->width * this->dlo_map->height)
  {
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlo_map, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub.publish(map_ros);

    pcl::io::savePLYFile("/home/zc/dlo_map.ply", *this->dlo_map);
  }
}

/**
 * Node Callback
 **/

void hj::MapNode::keyframeCB(const sensor_msgs::PointCloud2ConstPtr &keyframe)
{

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save keyframe to map
  this->map_stamp = keyframe->header.stamp;
  *this->dlo_map += *keyframe_pcl;

  if (!this->publish_full_map_)
  {
    if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height)
    {
      sensor_msgs::PointCloud2 map_ros;
      pcl::toROSMsg(*keyframe_pcl, map_ros);
      map_ros.header.stamp = ros::Time::now();
      map_ros.header.frame_id = this->odom_frame;
      this->map_pub.publish(map_ros);
    }
  }
}

bool hj::MapNode::savePcd(hj_slam::save_pcd::Request &req,
                          hj_slam::save_pcd::Response &res)
{

  pcl::PointCloud<PointType>::Ptr m =
      pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd" << "... ";
  std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
  res.success = ret == 0;

  if (res.success)
  {
    std::cout << "done" << std::endl;
  }
  else
  {
    std::cout << "failed" << std::endl;
  }

  return res.success;
}
