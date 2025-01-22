#pragma once
#include <assert.h>
#include <jsoncpp/json/json.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <queue>
#include "common_lib/gomros.h"
#include "common_lib/log.h"
#include "include/common/logger.h"
#include "include/config_struct.h"
#include "include/mapping_lib/carto_mapping/create_options.h"
#include "include/mapping_lib/mapping_interface.h"
#include "message_lib/imu_message.h"
#include "message_lib/odometer_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"  //这些头文件里包含对应的pb.h文件
#include "cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
enum SensorType {
  Lidar = 0,
  Odom = 1,
  Imu,
};

class CartoMapping : public MappingInterface {
 public:
  using Position = gomros::message::Position;
  using OdometerMessage = gomros::message::OdometerMessage;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using Logger = gomros::common::Logger;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;

  CartoMapping(const CartoMappingConfig &config);
  ~CartoMapping();

  virtual void SetConfiguration(void *config);
  // 处理里程计数据
  virtual void HandleOdomData(const OdometerMessage &data);
  // 处理雷达数据
  virtual void HandleLaserData(const RadarSensoryMessage &data);
  // 处理Imu数据
  virtual void HandleImuData(const ImuSensoryMessage &data);
  // 开始建图
  virtual void StartMapping();
  // 停止建图
  virtual void StopMapping(std::string map_name);
  virtual bool IsFinishMapping();

 private:
  void CreatCartoModule();

  void HandleTimeQueue();  //代替上面三个函数
  void FinishMapping();    // 结束建图函数
  void StopAndOptimize();
  void PaintMap();

  // 三个保存离线数据的函数
  void SaveLaserDataToFile(RadarSensoryMessage &data);
  void SaveOdomDataToFile(Position &pose);
  void SaveImuDataToFile(ImuSensoryMessage &data);

  // 三个读数据的函数
  void ReadRadarData();
  void ReadOdomData();
  void ReadImuData();

  //分割字符串函数
  std::vector<std::string> SplitCString(std::string &str, std::string delimit);
  // 数据格式转换并添加数据函数
  void OdomDataConversionAndAddFunc(Position &pose);
  void ImuDataConversionAndAddFunc(ImuSensoryMessage &data);
  void LaserDataConversionAndAddFunc(RadarSensoryMessage &data);

  cartographer::sensor::TimedPointCloudData ToCartoPointCloud(
      RadarSensoryMessage &data);
  cartographer::transform::Rigid3d ToCartoRigid3d(Position &pose);
  cartographer::sensor::ImuData ToCartoImu(ImuSensoryMessage &data);
  cartographer::common::Time ToCartoTime(uint64_t timestamp_us);

  CartoMappingConfig config_;

  // 建图接口MapBuilder，智能指针的声明（空指针）
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;

  // 路径接口指针TrajectoryBuilder，普通指针（空指针）
  cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder_;

  // 对数据列表操作时，加互斥锁，防止之间相互干扰
  // 换成std::mutex锁
  std::mutex time_queue_mtx_;
  std::mutex raw_lidar_list_mtx_;
  std::mutex raw_odom_list_mtx_;
  std::mutex raw_imu_list_mtx_;
  std::mutex paint_mutex_;
  std::mutex stop_mapping_mtx_;
  std::condition_variable stop_mapping_cond_;
  std::condition_variable data_cv_;

  // 传感器数据从文件中读完的标志位，保证读完才能进行下一步
  bool read_radar_finish_ = true;
  bool read_odom_finish_ = true;
  bool read_imu_finish_ = true;

  uint64_t latest_radar_timestamp_ = 0;
  uint64_t latest_odom_timestamp = 0;
  uint64_t latest_imu_timestamp_ = 0;
  uint64_t last_data_time_ = 0;

  bool first_laser_ = true;
  float laser_min_angle_;
  float laser_max_angle_;
  float laser_min_range_;
  float laser_max_range_;
  float laser_resolution_;

  float angle_increment_;

  int stepIncreament_ = 1;  // 倍数关系
  bool add_data_;

  // 三个结束处理数据（包括在线和离线）的标志位
  bool handle_radar_finish_ = true;
  bool handle_odom_finish_ = true;
  bool handle_imu_finish_ = true;

  // 三个文件打开/关闭标志位
  bool radar_file_open_ = false;
  bool odom_file_open_ = false;
  bool imu_file_open_ = false;

  //结束建图标志位
  bool finish_mapping_ = true;

  // 声明三个文件指针
  FILE *RadarDataFile_;
  FILE *OdomDataFile_;
  FILE *ImuDataFile_;

  // 创建三个缓存各种类型传感器数据的list容器
  std::list<RadarSensoryMessage> lidar_list_;  // List容器针对频繁存取的操作
  std::list<ImuSensoryMessage> imu_lists_;
  std::list<OdometerMessage> odom_list_;

  // 创建三个在读数据线程中存储从文件中读到的传感器数据的list容器
  std::list<RadarSensoryMessage> lidar_list_from_file_;
  std::list<Position> odom_list_from_file_;
  std::list<ImuSensoryMessage> imu_list_from_file_;

  // 创建一个存储各种传感器类型及时间戳的队列
  std::map<uint64_t, SensorType> time_queue_;

  int trajectory_id_;
  bool offline_mapping_;
  bool use_imu_;  // Carto里 有是否使用IMU的选择
  bool use_odom_;
  std::string map_name_;  // 地图名

  // 三个记录数据量的索引
  int record_index_radar_ = 0;
  int record_index_odom_ = 0;
  int record_index_imu_ = 0;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
