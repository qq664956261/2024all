/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer_ros/map_builder_bridge.h"

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.2;
constexpr double kConstraintMarkerScale = 0.025;
// 将 Cartographer 的 FloatColor 结构转换成 ROS 消息类型 std_msgs::ColorRGBA
::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

// 轨迹的Marker的声明与初始化
// 创建一个用于可视化特定轨迹的 ROS 标记（visualization_msgs::Marker）。
// 这样的标记可以在 RViz（即 ROS 可视化工具）中显示，帮助用户直观地看到机器人的路径或轨迹。
visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  // marker.ns（命名空间）: 为标记设置一个唯一的命名空间，这里使用 "Trajectory " 加上轨迹的 ID 来确保每个轨迹的标记都是独立的。
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);
  // marker.id: 设置标记的 ID，由于轨迹的每个 Marker 在命名空间中已经是唯一的了，这里可以简单地将 ID 设为 0
  marker.id = 0;
  // note: Marker::LINE_STRIP 它会在每两个连续的点之间画一条线 eg: 0-1, 1-2, 2-3
  // 指定标记类型为 LINE_STRIP，这意味着 RViz 将在连续的点之间绘制线条，用于表示轨迹。
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  // 设置标记的时间戳为当前时间，::ros::Time::now()。
  marker.header.stamp = ::ros::Time::now();
  // marker.header.frame_id: 设置标记所在的坐标帧，这通常是一个全局坐标系，如 "map"
  marker.header.frame_id = frame_id;
  // 调用 ToMessage 函数将 Cartographer 的颜色转换为 ROS 的 std_msgs::ColorRGBA
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  // 设置线条的宽度。kTrajectoryLineStripMarkerScale 是一个预定义的常量，用于定义线条的线宽。
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  // 虽然 LINE_STRIP 类型的标记主要是依靠其 points 数组来定义路径的，但还可以设置其全局位姿。这里将标记的 orientation.w 设为 1（表示没有旋转），
  // 并将 position.z 设为 0.05，可能是为了确保线条稍微漂浮在地图表面上，以避免 Z 战斗问题
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

// 获取landmark的index,如果在unordered_map找到就返回index, 如果找不到就以map的个数新建个index
// 在一个给定的关联容器（这里是 std::unordered_map<std::string, int> 类型）中检索特定地标标识符（landmark_id）对应的索引。
// 如果标识符已经存在于容器中，它会返回这个地标的当前索引。如果地标标识符尚未存在，函数会将它与一个新索引关联后加入到容器中，然后返回这个新索引
int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

// landmark的Marker的声明与初始化
visualization_msgs::Marker CreateLandmarkMarker(int landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = "Landmarks";
  marker.id = landmark_index;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

// 将marker添加到markers后边,并清空marker
void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace
// 这个类充当了 Cartographer 核心库和 ROS 之间的桥梁，处理从 ROS 消息到 Cartographer 内部数据结构的转换，同时对外提供了 ROS 服务和话题接口
/**
 * @brief 根据传入的node_options, MapBuilder, 以及tf_buffer 完成三个本地变量的初始化
 * 
 * @param[in] node_options 参数配置
 * @param[in] map_builder 在node_main.cc中传入的MapBuilder
 * @param[in] tf_buffer tf_buffer
 */
MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}

// 加载pbstream文件
// 用于加载保存的 SLAM 状态的函数。Cartographer 支持将其 SLAM 进程的状态序列化到文件中，
// 此功能即用于从这样的文件中恢复状态。此功能对于暂停和重新启动地图构建过程、以及加载先前计算好的地图非常有用。
void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  // 检查后缀是否是.pbstream
  // 函数首先检查传入的文件名 (state_filename) 是否具有正确的后缀（".pbstream"）。
  // 这是 Cartographer 用于序列化和反序列化其状态的专用格式。如果文件名不以 ".pbstream" 结尾，则终止执行
  // substr 是 std::string 类的一个成员函数，它被用来从字符串中提取一个子字符串。substr 函数接受两个参数：pos 和 len。
  // 其中 pos 是起始位置（从0开始计数），表示从哪个位置开始提取子字符串；len 是长度，表示要提取的字符数
//   state_filename.size() - suffix.size() 计算了后缀开始的位置索引。举例来说，如果 state_filename 是 "example.pbstream"，
//   那么这个计算结果是文件名总长度减去后缀 ".pbstream" 的长度，即后缀的起始位置。
// std::max<int>(... , 0) 确保即使 state_filename 的长度小于 suffix 的长度，也不会产生负数，这种情况下会使用 0。
// 尽管在这个上下文中，这个检查可能看起来多余（因为在常规使用中文件名通常会比 ".pbstream" 长），但它是一种良好的编程实践，可以防止字符串下标越界。
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  // 加载文件内容
  // 一旦确认文件后缀正确，函数会记录开始加载状态文件的消息。
//   使用 ProtoStreamReader 对象 stream 打开指定的文件。ProtoStreamReader 是 Cartographer 提供的用于从 protobuf 流读取数据的工具。
// 调用 LoadState 方法加载状态。此方法接受前面创建的 stream 对象和 load_frozen_state 参数。load_frozen_state 指示 Cartographer 是否应该停止更新加载的轨迹（即将轨迹“冻结”）。
  cartographer::io::ProtoStreamReader stream(state_filename);
  // 解析数据
  map_builder_->LoadState(&stream, load_frozen_state);
}

// 开始一条新轨迹
int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  // Step: 1 开始一条新的轨迹, 返回新轨迹的id,需要传入一个函数
  //   expected_sensor_ids：这是一个包含所有期望传感器ID的集合，用于此轨迹。
  //   在 SLAM 中，不同的传感器（如激光雷达、IMU等）会提供不同的数据流，这些数据流是定位和地图构建的关键。
  // trajectory_options.trajectory_builder_options：包含了轨迹的配置选项，这些选项定义了如何处理传感器数据以及SLAM的其他重要参数。
  // 一个 lambda 表达式：用作Local SLAM结果的回调。当从传感器数据计算出一个新的局部位姿时，这个回调会被调用。
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      // lambda表达式 local_slam_result_callback_
      [this](const int trajectory_id, 
             const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             ::cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        // 保存local slam 的结果数据 5个参数实际只用了4个
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  // 检查sensor_bridges_容器（一个存储了SensorBridge对象的映射）以确认没有其它轨迹具有相同的ID。
  // SensorBridge是处理特定类型传感器数据的类，它负责将ROS的消息类型转换为Cartographer可以理解的形式。
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  // Step: 2 为这个新轨迹 添加一个SensorBridge
  // 创建SensorBridge：为每条轨迹创建一个对应的SensorBridge对象。这一步需要提供几个参数，
  // 包括对激光雷达数据进行上采样或下采样的参数、用来指定SLAM框架中坐标转换的追踪帧、
  // SLAM系统的TF查询超时设置以及对应轨迹的TrajectoryBuilder指针。
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, 
      tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id)); // CollatedTrajectoryBuilder
  
  // Step: 3 保存轨迹的参数配置
  // 存储轨迹配置：将轨迹配置选项添加到trajectory_options_映射中，用轨迹ID作为键。这一步确保了能够随时访问特定轨迹的配置选项。
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

// 结束指定id的轨迹
void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

// 当所有的轨迹结束时, 执行一次全局优化
void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

// 将地图, 轨迹, 以及各个传感器数据进行序列化保存
bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
  return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                            filename);
}

/**
 * @brief 获取对应id轨迹的 索引为 submap_index 的地图的栅格值及其他信息
 * 
 * @param[in] request 轨迹id与submap的index
 * @param[in] response 是否成功
 */
// 此函数HandleSubmapQuery是MapBuilderBridge类中用于处理ROS服务请求的成员函数，旨在响应对特定子图查询的请求。
// 在Cartographer中，子图是构建SLAM地图的基本组件，代表了机器人在一定时间和空间范围内的局部地图。
// 请求(Request): 包含两个主要字段trajectory_id和submap_index。trajectory_id指示哪条轨迹的子图被查询，而submap_index是在该轨迹中需要查询的特定子图的索引。
// 响应(Response): 包括子图的版本、一组描述子图内容的纹理以及状态信息。纹理中包含了子图的栅格数据、尺寸、分辨率和位置。
void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  // 构造子图ID: 首先根据请求中提供的轨迹ID和子图索引构造一个子图ID（SubmapId）。
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  // 获取压缩后的地图数据
  // 获取子图数据: 调用SubmapToProto函数获取请求的子图信息。如果查询成功，该函数返回空字符串；如果失败，返回错误信息
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }
  // 子图版本号：将从response_proto中获取的子图版本信息存储到响应消息中。
  response.submap_version = response_proto.submap_version();

  // 将response_proto中的地图栅格值存入到response中
  // 纹理数据：遍历response_proto中的所有纹理数据，为每个纹理创建对应的存储结构并填充相关属性（如栅格数据单元、尺寸、分辨率等），同时将子图的位姿转换为ROS消息格式存入。
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    // 获取response中存储地图变量的引用
    auto& texture = response.textures.back();
    // 对引用的变量进行赋值
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}

// 向pose graph中添加新的active状态的轨迹, 并返回所有的轨迹状态
// 目的是获取并返回当前SLAM系统中所有轨迹的状态信息。这是在使用Cartographer做SLAM时的一个重要功能，因为它允许用户查询每个轨迹是否处于活动、完成或冻结状态。
std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  // 获取Pose Graph中的轨迹状态：首先通过调用 map_builder_->pose_graph()->GetTrajectoryStates() 获取所有已知轨迹（已经注册到 Pose Graph 中的轨迹）的状态。
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  // 为活跃的轨迹添加active状态, 如果trajectory_states中存在这个轨迹id,则会被忽略不会被添加进去
  // 补充活跃轨迹的状态：由于可能存在一些还未添加到 Pose Graph 的活跃轨迹（例如，可能正在等待输入的传感器数据），
  // 这些轨迹已经通过 SensorBridge 对象进行了注册，没有被直接反映在 Pose Graph 的状态返回值中。
  // 所以，对于 sensor_bridges_ 容器中的每一项（每个条目代表一个轨迹及其对应的 SensorBridge），
  // 如果轨迹ID尚未出现在 trajectory_states 中，就将其作为活跃轨迹加入 trajectory_states 集合。
  for (const auto& sensor_bridge : sensor_bridges_) {
    trajectory_states.insert(std::make_pair(
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE));
  }
  // 函数最终返回一个包含所有轨迹和它们状态的映射。状态可能是ACTIVE、FINISHED或FROZEN等，这取决于轨迹的当前活动性、是否完成数据处理、或者是否被处于"冻结"状态以备以后可能的进一步操作。
  return trajectory_states;
}

// 获取所有submap的信息, 包括 trajectory_id,submap_index,submap_version,pose
// 函数用于获取Cartographer中所有子图的状态列表，并包装成适用于ROS的消息格式。
cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  // 初始化ROS消息：创建一个 cartographer_ros_msgs::SubmapList 类型的对象 submap_list，用于存储所有子图的信息。
  cartographer_ros_msgs::SubmapList submap_list;
  // stamp 字段被设置为当前时间，相当于调用时刻。
  submap_list.header.stamp = ::ros::Time::now();
  // frame_id 字段被设置为配置中定义的地图坐标系，node_options_.map_frame。
  submap_list.header.frame_id = node_options_.map_frame;
  // 遍历所有子图：调用Cartographer的 PoseGraph 接口的 GetAllSubmapPoses 方法遍历当前所有的子图和它们的位姿。
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    // 对于每个子图位姿，创建一个 cartographer_ros_msgs::SubmapEntry 类型的对象来保存子图信息。
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(
        submap_id_pose.id.trajectory_id);
    // 设置 submap_entry 的字段值，包括：
    // is_frozen 标志指示对应轨迹是否已冻结不再接受新数据。
    // trajectory_id 和 submap_index 提供子图的唯一标识。
    // submap_version 指示该子图最后更新的版本。
    // pose 字段存储了子图在全局坐标系下的位姿。
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

// 获取local坐标系下的TrajectoryData
//  函数负责从 MapBuilderBridge 类中检索所有轨迹的本地轨迹数据。本地轨迹数据包括了SLAM进程中每个轨迹的最新位姿和相关的传感器数据。
std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  // 创建一个 std::unordered_map，将轨迹ID映射为 LocalTrajectoryData 结构，用于存储每个轨迹的相关数据。
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  // 遍历 sensor_bridges_，这是一个存储轨迹ID与其对应 SensorBridge 对象之间关联的容器。
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    // 获取local slam 数据
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      // 使用互斥锁 mutex_ 保护对 local_slam_data_ 的访问，
      absl::MutexLock lock(&mutex_);
      // 检查 local_slam_data_ 容器是否包含当前轨迹ID的条目。如果没有，则跳过此轨迹。
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      // 读取local_slam_data_要上锁
      // 访问并存储指向该轨迹本地SLAM数据的shared_ptr
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    // 确认 trajectory_options_ 中存在轨迹配置项，这是SLAM为每个轨迹设置的特定参数
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);

    // 填充LocalTrajectoryData
    // 填充 LocalTrajectoryData 结构，包含局部SLAM数据，以及以下坐标变换：
    local_trajectory_data[trajectory_id] = {
        local_slam_data,

        // local frame 到 global frame间的坐标变换
        // 从局部坐标系到全局坐标系的变换，使用 map_builder_ 对象的 pose_graph() 方法获取。
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),

        // published_frame 到 tracking_frame 间的坐标变换
        // 计算在特定时间点，published_frame 坐标系到 tracking_frame 坐标系的变换，使用 sensor_bridge 对象的 tf_bridge() 方法获取。
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),

        trajectory_options_[trajectory_id]};
  } // end for
  return local_trajectory_data;
}

// 获取对应id轨迹的所有位姿的集合、
// 处理来自其他ROS节点的轨迹查询请求。本方法的主要目的是提供一个特定轨迹的所有节点的位姿信息
void MapBuilderBridge::HandleTrajectoryQuery(
    cartographer_ros_msgs::TrajectoryQuery::Request& request,
    cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
  // map_builder_->pose_graph()->GetTrajectoryNodePoses() 方法获取地图构建器中所有轨迹节点的位姿信息。
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // 遍历指定轨迹的所有节点：使用 request.trajectory_id 作为索引，在上一步获得的所有节点位姿中过滤出属于请求轨迹ID的那些节点
  for (const auto& node_id_data :
       node_poses.trajectory(request.trajectory_id)) {
    // 检查位姿数据的有效性：如果节点数据中不含有有效的常数位姿信息（constant_pose_data），则跳过该节点。
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    // 为每个有效节点创建一个 geometry_msgs::PoseStamped 消息，这种消息类型包含了节点的时间戳（stamp）和在全局坐标系下的位姿（pose）。
    geometry_msgs::PoseStamped pose_stamped;
    // 设置消息头信息：包括指定消息的坐标帧（frame_id）为地图帧（node_options_.map_frame）和将节点的时间转换为ROS时间格式（stamp）。
    pose_stamped.header.frame_id = node_options_.map_frame;
    pose_stamped.header.stamp =
        ToRos(node_id_data.data.constant_pose_data.value().time);
    // 使用的是global坐标系下的坐标
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    // 追加到响应列表：将上述创建的位姿消息追加到响应消息的 trajectory 数组中。
    response.trajectory.push_back(pose_stamped);
  }
  // 设置响应状态：最后，设置响应消息的状态为 OK，并添加一条包含检索到的节点数量的状态消息。
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = absl::StrCat(
      "Retrieved ", response.trajectory.size(),
      " trajectory nodes from trajectory ", request.trajectory_id, ".");
}

/**
 * @brief 获取所有的轨迹节点与约束的rviz可视化MarkerArray
 * @return visualization_msgs::MarkerArray 返回marker的集合
 */
// 函数的目的是构建一个MarkerArray消息，该消息包含了所有轨迹节点的视觉表示，以便可以在ROS的RViz工具中进行可视化。
// MarkerArray是一组Marker消息，每个Marker代表了SLAM算法在不同时间点记录的机器人的位置（节点）。
visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;

  // 获取节点位姿信息
  // 获取所有轨迹节点的位置：通过 GetTrajectoryNodePoses 获取每个轨迹节点的位置信息。
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.

  // 同一条轨迹内 最后一个子图间约束的 节点的索引
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  // 不同轨迹 最后一个子图间约束的 节点的索引
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  // 初始化为0
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }

  const auto constraints = map_builder_->pose_graph()->constraints();
  // 找到所有轨迹的最后一个INTER_SUBMAP约束的node_index
  for (const auto& constraint : constraints) {
    // 是外部子图关系才往下走
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
      
      // 找到当前时刻 同一轨迹下的最后一个inter_submap约束的node_index
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } 
      // 不同轨迹下的最后一个inter_submap的node_index
      else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  // 遍历不同的轨迹来生成marker
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);

    // 将刚才找到的当前时刻的最后一个inter_submap约束的节点索引 取出来
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    
    // 节点索引的最大值
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    // 如果轨迹结束了, 更新节点的索引为 轨迹的最后一个节点的索引
    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }

    marker.color.a = 1.0;
    
    // 遍历所有节点
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      // 如果没有位姿数据就先跳过
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }

      // 将 节点在 global map 下的坐标 放入marker中
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      // 更新到inter_trajectory_constrained_node, 就将color设置成0.5
      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }

      // 更新到inter_submap_constrained_node, 就将color设置成0.25
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      // 通过将轨迹分成多个标记来解决RViz中16384点的限制.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    } // end for
    
    // 将剩余marker放入trajectory_node_list.markers中
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    // 如果该轨迹id不在trajectory_to_highest_marker_id_中, 将current_last_marker_id保存
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } 
    else {
      marker.action = visualization_msgs::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      // 更新last_marker_id
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }

  } // end for
  return trajectory_node_list;
}

// landmark 的rviz可视化设置
visualization_msgs::MarkerArray MapBuilderBridge::GetLandmarkPosesList() {
  visualization_msgs::MarkerArray landmark_poses_list;
  // 获取landmark poses
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame));
  }
  return landmark_poses_list;
}

/**
 * @brief 获取位姿图中所有的约束,分成6种类型,放入不同类型的marker中
 * 
 * @return visualization_msgs::MarkerArray 返回6种marker的集合
 */
visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;

  // 6种marker的声明

  // 1 内部子图约束, 非全局约束, rviz中显示的最多的约束
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  // note: Marker::LINE_LIST: 每对点之间画一条线, eg: 0-1, 2-3, 4-5
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  // 2 Intra residuals
  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  // 将该标记和其他数量较少的标记设置z为略高于帧内约束标记, 以确保它们可见.
  residual_intra_marker.pose.position.z = 0.1;

  // 3 Inter constraints, same trajectory, rviz中显示的第二多的约束
  // 外部子图约束, 回环约束, 全局约束
  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  // 4 Inter residuals, same trajectory
  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  // 5 Inter constraints, different trajectories
  visualization_msgs::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  // 6 Inter residuals, different trajectories
  visualization_msgs::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();

  // 将约束信息填充到6种marker里
  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;

    // 根据不同情况,将constraint_marker与residual_marker 指到到不同的maker类型上

    // 子图内部的constraint,对应第一种与第二种marker
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      // 各种轨迹的子图的颜色映射-添加轨迹ID以确保不同的起始颜色 还要添加25的固定偏移量, 以避免与轨迹具有相同的颜色. 
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } 
    else {
      // 相同轨迹内,子图外部约束, 对应第三种与第四种marker
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow 亮黄色
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } 
      // 不同轨迹间的constraint,对应第五种与第六种marker
      else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan 亮青色
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    // 设置颜色信息
    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    // 在submap_poses中找到约束对应的submap_id
    const auto submap_it = submap_poses.find(constraint.submap_id);
    // 没找到就先跳过
    if (submap_it == submap_poses.end()) {
      continue;
    }
    // 子图的坐标
    const auto& submap_pose = submap_it->data.pose;

    // 在trajectory_node_poses中找到约束对应的node_id
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    // 节点在global坐标系下的坐标
    const auto& trajectory_node_pose = node_it->data.global_pose;
    // 根据子图坐标与约束的坐标变换算出约束的另一头的坐标
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    // 将子图与节点间的约束放进不同类型的marker中
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    // 两种方式计算出的节点坐标不会完全相同, 将这个差值作为残差发布出来
    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  } // end for

  // 将填充完数据的Marker放到MarkerArray中
  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

// 获取对应轨迹id的SensorBridge的指针
SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

/**
 * @brief 保存local slam 的结果
 * 
 * @param[in] trajectory_id 当前轨迹id
 * @param[in] time 扫描匹配的时间
 * @param[in] local_pose 扫描匹配计算出的在local坐标系下的位姿
 * @param[in] range_data_in_local 扫描匹配使用的雷达数据
 */
// OnLocalSlamResult 是一个回调函数，当 Cartographer 的局部 SLAM 进程（通常包括将传感器数据，比如激光扫描，匹配到局部子图）完成一次迭代时被调用。
void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local) {
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time, local_pose,
                                             std::move(range_data_in_local)});
  // 保存结果数据
  absl::MutexLock lock(&mutex_);
  local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

// lx add 获取节点位姿与雷达数据
std::shared_ptr<MapById<NodeId, TrajectoryNode>> MapBuilderBridge::GetTrajectoryNodes() {
  std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
      std::make_shared<MapById<NodeId, TrajectoryNode>>(
        map_builder_->pose_graph()->GetTrajectoryNodes());
  return trajectory_nodes;
}

}  // namespace cartographer_ros