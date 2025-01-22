#include "slam_catkin.h"
#include "utility_ros1.h"
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>


LocalizerCatkin::LocalizerCatkin(std::string config_file, ros::NodeHandle &node):
    nh(node)
{
  localizer = Localizer(config_file);

}

LocalizerCatkin::~LocalizerCatkin()
{

}

MapperCatkin::MapperCatkin(std::string config_file, ros::NodeHandle &node):
    nh(node)
{
  mapper = Mapper(config_file);

}

MapperCatkin::~MapperCatkin()
{

}
