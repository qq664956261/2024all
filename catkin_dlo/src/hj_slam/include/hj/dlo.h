/************************************************************
 *
// created by: zhangcheng
 *
 ***********************************************************/

#include <atomic>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <thread>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <hj_slam/save_pcd.h>
#include <hj_slam/save_traj.h>
#include <nano_gicp/nano_gicp.hpp>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <laser_geometry/laser_geometry.h>
#include "hj_interface/Pose.h"
#include "hj_interface/Imu.h"
#include "function_factory.h"
#include "node_factory.h"
#include "hj/ExtrinsicErrorTerm.hh"
#include "hj/lidarFactor.h"
#include "hj/scancontext.h"
#include "hj/CLaserOdometry2D.h"
#include "hj/extended_kalman_filter.h"
#include "hj/point_undistort.h"
typedef pcl::PointXYZI PointType;

namespace hj {

  class OdomNode;
  class MapNode;

}
