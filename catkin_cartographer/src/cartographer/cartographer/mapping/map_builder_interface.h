/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_

#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {
// MapBuilderInterface 的定义，是一个抽象接口，负责实现地图构建（SLAM）的核心逻辑，支持动态创建轨迹构建器（TrajectoryBuilder）、管理轨迹、序列化/反序列化 SLAM 状态以及其他相关操作。
// MapBuilderInterface 是一个纯虚类，定义了地图构建所需的主要接口，具体功能需由子类实现。
proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// This interface is used for both library and RPC implementations.
// Implementations wire up the complete SLAM stack.
class MapBuilderInterface {
 public:
  using LocalSlamResultCallback =
      TrajectoryBuilderInterface::LocalSlamResultCallback;

  using SensorId = TrajectoryBuilderInterface::SensorId;

  MapBuilderInterface() {}
  //定义了一个虚析构函数，确保派生类在删除基类指针时，能正确调用派生类的析构函数
  virtual ~MapBuilderInterface() {}
  //禁止拷贝构造和拷贝赋值操作。
  //表示该类的对象是 不可拷贝的，也不能通过赋值操作产生副本。
  MapBuilderInterface(const MapBuilderInterface&) = delete;
  MapBuilderInterface& operator=(const MapBuilderInterface&) = delete;

  // Creates a new trajectory builder and returns its index.
  //纯虚函数，定义了向地图构建器添加轨迹构建器（TrajectoryBuilder）的接口
//   类型：std::set<SensorId>。
//   含义：预期的传感器 ID 集合，用于指定当前轨迹将处理哪些传感器数据。例如，IMU、激光雷达或里程计的 ID。
//   作用：确保轨迹构建器只接收与其相关的传感器数据。
//   类型：proto::TrajectoryBuilderOptions。
//   含义：轨迹构建器的配置选项，定义了与轨迹构建相关的参数。
//   典型内容：
//   是否启用 2D 或 3D 轨迹构建器。
//   激光雷达和 IMU 的参数。
//   地图更新频率。
//   作用：通过指定的配置选项，创建定制化的轨迹构建器
//   local_slam_result_callback:
//   类型：LocalSlamResultCallback。
//   含义：回调函数，用于处理局部 SLAM 结果（如优化后的位姿、子地图数据等）。
//   作用：提供实时反馈，便于应用程序对局部 SLAM 结果进行进一步处理
//= 0 用于定义纯虚函数。纯虚函数是一个没有实现的函数，表示该函数必须由子类提供具体实现
  virtual int AddTrajectoryBuilder(
      const std::set<SensorId>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) = 0;

  // Creates a new trajectory and returns its index. Querying the trajectory
  // builder for it will return 'nullptr'.
  //目的：创建一个新的轨迹对象，用于加载和反序列化已有的轨迹数据。
  //返回值：返回新创建轨迹的唯一索引（trajectory_id）。
  //特殊性：对于此函数创建的轨迹，无法通过 GetTrajectoryBuilder() 获取有效的轨迹构建器，会返回 nullptr。
  //原因是这类轨迹仅用于反序列化，不需要实时接收传感器数据和构建过程
  //类型：proto::TrajectoryBuilderOptionsWithSensorIds
  //含义：包含轨迹的反序列化配置选项以及相关传感器信息
  virtual int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds&
          options_with_sensor_ids_proto) = 0;
  //目的：根据 trajectory_id 获取与之关联的 TrajectoryBuilderInterface 对象。
  //返回值：
  //如果指定的 trajectory_id 有对应的轨迹构建器，则返回其指针。
  //如果指定的 trajectory_id 没有对应的构建器（例如通过反序列化加载的轨迹），则返回 nullptr。
  // Returns the 'TrajectoryBuilderInterface' corresponding to the specified
  // 'trajectory_id' or 'nullptr' if the trajectory has no corresponding
  // builder.
  virtual mapping::TrajectoryBuilderInterface* GetTrajectoryBuilder(
      int trajectory_id) const = 0;
  //目的：标记指定的轨迹构建器为完成状态（finished）。
  //行为：
  //一旦标记为完成状态，该轨迹将不再接收新的传感器数据。
  //通常会触发相关的清理操作（如优化或释放资源）。
  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Fills the SubmapQuery::Response corresponding to 'submap_id'. Returns an
  // error string on failure, or an empty string on success.
  //目的：将指定的子地图（Submap）数据转换为序列化的 proto::SubmapQuery::Response 格式，以便外部系统使用。
  //返回值：
  //成功：返回空字符串（""）。
  //失败：返回包含错误信息的字符串。
  //函数参数
  //submap_id:
  //类型：SubmapId
  //含义：唯一标识子地图的 ID。
  //结构：
  //通常由轨迹 ID 和子地图索引组成（如 {trajectory_id, submap_index}）。
  //作用：指定要查询的子地图。
  //response:
  //类型：proto::SubmapQuery::Response*
  //含义：用于存储子地图的序列化响应。
  //内容：
  //包含子地图的网格数据（如栅格地图或点云）。
  //包括子地图的元数据（如分辨率、边界等）。
  virtual std::string SubmapToProto(const SubmapId& submap_id,
                                    proto::SubmapQuery::Response* response) = 0;

  // Serializes the current state to a proto stream. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  //目的：将当前 SLAM 系统的状态序列化为一个 proto 流，便于保存和恢复。
  //行为：
  //如果 include_unfinished_submaps 为 true，序列化数据会包含未完成的子地图（即仍可能接收传感器数据的子地图）。
  //通过 writer 接口将序列化后的数据写入目标流中。
  //函数参数
  //include_unfinished_submaps:
  //类型：bool
  //含义：
  //true：将未完成的子地图包含在序列化状态中。
  //false：仅序列化完成的子地图。
  //作用：控制是否序列化未完成子地图，影响输出的完整性和一致性。
  //writer:
  //类型：io::ProtoStreamWriterInterface*
  //含义：用于写入 proto 数据流的接口。
  //功能：
  //定义了将序列化数据写入文件或其他存储设备的具体操作
  virtual void SerializeState(bool include_unfinished_submaps,
                              io::ProtoStreamWriterInterface* writer) = 0;

  // Serializes the current state to a proto stream file on the host system. If
  // 'include_unfinished_submaps' is set to true, unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included in the serialized state.
  // Returns true if the file was successfully written.
  //目的：将当前 SLAM 系统的状态序列化为 proto 文件，保存到本地文件系统。
  //返回值：
  //true：文件成功写入。
  //false：写入失败（例如文件路径无效或没有写入权限）。
  virtual bool SerializeStateToFile(bool include_unfinished_submaps,
                                    const std::string& filename) = 0;

  // Loads the SLAM state from a proto stream. Returns the remapping of new
  // trajectory_ids.
  //目的：从 proto 数据流中加载 SLAM 系统的状态，恢复轨迹、子地图和位姿图等内容。
  //返回值：
  //一个映射关系：{proto_trajectory_id -> new_trajectory_id}。
  //将序列化文件中的轨迹 ID 映射到系统加载后的新轨迹 ID，确保轨迹在新系统中能正确匹配。
  //函数参数
  //reader:
  //类型：io::ProtoStreamReaderInterface*
  //含义：读取 proto 数据流的接口。
  //作用：
  //提供从文件或其他输入源读取序列化状态的功能。
  //读取内容包括轨迹、子地图和位姿图的序列化数据。
  //load_frozen_state:
  //类型：bool
  //含义：
  //true：加载冻结的 SLAM 状态（不再优化）。
  //false：加载状态后允许继续更新和优化。
  //作用：
  //控制加载的状态是否继续接收传感器数据或进行后续 SLAM 处理。
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadState(io::ProtoStreamReaderInterface* reader, bool load_frozen_state) = 0;

  // Loads the SLAM state from a pbstream file. Returns the remapping of new
  // trajectory_ids.
  //目的：从一个本地的 .pbstream 文件中加载 SLAM 系统状态，包括轨迹、子地图和位姿图。
  //返回值：
  //返回一个映射：{proto_trajectory_id -> new_trajectory_id}。
  //该映射表示 .pbstream 文件中的轨迹 ID 映射到加载后系统中新分配的轨迹 ID。
  virtual std::map<int /* trajectory id in proto */, int /* trajectory id */>
  LoadStateFromFile(const std::string& filename, bool load_frozen_state) = 0;
  //目的：返回当前 SLAM 系统中活动的轨迹构建器（TrajectoryBuilder）的数量。
  //返回值：
  //一个整数，表示已创建的轨迹构建器数量。
  virtual int num_trajectory_builders() const = 0;
  //目的：返回当前 SLAM 系统中的 位姿图接口（PoseGraphInterface），用于访问或操作位姿图（Pose Graph）。
  //返回值：
  //一个指向 PoseGraphInterface 的指针，表示系统中管理轨迹、节点和边（子地图和位姿之间关系）的核心图结构。
  virtual mapping::PoseGraphInterface* pose_graph() = 0;
  //目的：获取所有轨迹构建器的配置选项以及与之关联的传感器 ID。
  //返回值：
  //返回一个 std::vector 的引用，元素类型为 proto::TrajectoryBuilderOptionsWithSensorIds。
  //每个元素表示一个轨迹构建器的配置信息以及该轨迹使用的传感器。
  virtual const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>&
  GetAllTrajectoryBuilderOptions() const = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_INTERFACE_H_
