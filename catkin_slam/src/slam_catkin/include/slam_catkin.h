#include "Localizer.h"
#include "Mapper.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

class LocalizerCatkin
{
public:
  LocalizerCatkin(std::string config_file, ros::NodeHandle &node);
  virtual ~LocalizerCatkin();

protected:
  ros::NodeHandle &nh;
  Localizer localizer;
};

class MapperCatkin
{
public:
  MapperCatkin(std::string config_file, ros::NodeHandle &node);
  virtual ~MapperCatkin();

protected:
  ros::NodeHandle &nh;
  Mapper mapper;
};
