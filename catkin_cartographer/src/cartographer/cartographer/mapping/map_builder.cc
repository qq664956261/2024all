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

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
  
// c++11: 匿名命名空间, 作用域被限制在本文件内
namespace { 

using mapping::proto::SerializedData;

// 只返回传感器类型是RANGE的topic的集合
//从传感器 ID 集合 expected_sensor_ids 中筛选出所有类型为 RANGE 的传感器 ID，并返回它们的名称（id）。
std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

// 检查是否是纯定位模式,支持2种纯定位的参数名字,老参数已经弃用,会报警告但程序不会终止
//根据 TrajectoryBuilderOptions 配置，为指定的轨迹（trajectory_id）添加一个 PureLocalizationTrimmer。
//PureLocalizationTrimmer 是一个子地图管理器，作用是限制轨迹中的子地图数量，常用于纯定位模式（pure localization）
void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    PoseGraph* pose_graph) {
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

/**
 * @brief 保存配置参数, 根据给定的参数初始化线程池, 并且初始化pose_graph_与sensor_collator_
 * 
 * @param[in] options proto::MapBuilderOptions格式的 map_builder参数
 */
//目的
//初始化 MapBuilder 对象，设置线程池、位姿图（PoseGraph2D 或 PoseGraph3D）、传感器数据协同器等 SLAM 系统的核心组件
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) { // param: num_background_threads
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());

  // 2d位姿图(后端)的初始化
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  // 3d位姿图(后端)的初始化
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  } 

  // 在 cartographer/configuration_files/map_builder.lua 中设置
  // param: MAP_BUILDER.collate_by_trajectory 默认为false
  // sensor_collator_ 作为一个类的成员变量，能够根据条件被初始化为不同的类型对象（sensor::TrajectoryCollator 或 sensor::Collator），是因为这两个类型共享一个共同的基类或接口，通常通过多态来实现
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    // sensor_collator_初始化, 实际使用这个
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}

/**
 * @brief 创建一个新的 TrajectoryBuilder 并返回它的 trajectory_id
 * 
 * @param[in] expected_sensor_ids 所有需要的topic的名字的集合
 * @param[in] trajectory_options 轨迹的参数配置
 * @param[in] local_slam_result_callback 需要传入的回调函数
 *        实际上是map_builder_bridge.cc 中的 OnLocalSlamResult() 函数
 * @return int 新生成的轨迹的id
 */
// 该函数的主要作用是创建一个新的轨迹构建器（TrajectoryBuilder），并将其与对应的位姿图（PoseGraph）关联，同时设置轨迹的相关属性和初始位姿
// 函数功能
// 轨迹 ID 的分配：
// 使用 trajectory_builders_.size() 确定新的轨迹 ID，确保每个轨迹都有唯一的标识符。
// 运动过滤器的初始化（可选）：
// 如果配置中指定了 pose_graph_odometry_motion_filter，则创建一个 MotionFilter，过滤掉微小的运动更新。
// 轨迹构建器的创建：
// 根据配置决定使用 2D 或 3D 的轨迹构建器：
// 2D 构建器：LocalTrajectoryBuilder2D。
// 3D 构建器：LocalTrajectoryBuilder3D。
// 构建器初始化包括：
// 点云预处理（如下采样）。
// 子地图管理。
// 激光点云与位姿的匹配。
// 全局轨迹构建器的创建：
// 使用 CollatedTrajectoryBuilder 将前端构建器与后端位姿图结合。
// 提供回调函数处理局部 SLAM 结果。
// 添加纯定位模式（可选）：
// 如果启用了 pure_localization 或 pure_localization_trimmer，添加对应的子地图修剪器，限制子地图数量。
// 设置初始位姿（可选）：
// 如果配置中提供了初始位姿，设置该轨迹在位姿图中的初始位姿。
// 保存轨迹配置信息：
// 将 expected_sensor_ids 和 trajectory_options 保存到 all_trajectory_builder_options_，便于后续使用和状态序列化。
// 返回新轨迹的 ID：
// 返回创建的轨迹的唯一标识符。
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {

  // id是从零开始的, 所以新trajectory_id就是trajectory_builders_的size()
  // 使用 trajectory_builders_.size() 确定新的轨迹 ID，确保每个轨迹都有唯一的标识符。
  const int trajectory_id = trajectory_builders_.size();

  // 运动过滤器, 运动太小没必要进行更新
  // 配置文件中没有 pose_graph_odometry_motion_filte
  
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;

  // LOG(INFO) << "pose_graph odometry_motion_filter is " << trajectory_options.has_pose_graph_odometry_motion_filter();
  // 上面会打印出0, 所以没有使用后端的里程计的motion_filter
  // 如果配置中指定了 pose_graph_odometry_motion_filter，则创建一个 MotionFilter，过滤掉微小的运动更新
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  // LocalTrajectoryBuilder 就是前端, 不带 Loop Closure 
  // 包含了 Pose Extrapolator, Scan Matching, 生成submap 等

  // 3d的轨迹
  if (options_.use_trajectory_builder_3d()) {
    // local_trajectory_builder(前端)的初始化
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    } 

    /**
     * c++11: static_cast关键字（编译时类型检查）: static_cast < type-id > ( expression )
     * 该运算符把expression转换为type-id类型, 但没有运行时类型检查来保证转换的安全性
      （1）用于基本数据类型之间的转换, 如把int转换为char, 把int转换成enum, 
      （2）把空指针转换成目标类型的空指针
      （3）把任何类型的表达式类型转换成void类型
      （4）用于类层次结构中父类和子类之间指针和引用的转换.

     * c++11: dynamic_cast关键字（运行时类型检查）: dynamic_cast < type-id > ( expression )
        该运算符把 expression 转换成 type-id 类型的对象. Type-id必须是类的指针、类的引用或者void *
        如果type-id是类指针类型, 那么expression也必须是一个指针
        如果type-id是一个引用, 那么expression也必须是一个引用

        dynamic_cast主要用于类层次间的上行转换（子类到父类）和下行转换（父类到子类）, 还可以用于类之间的交叉转换.
        在类层次间进行上行转换时, dynamic_cast和static_cast的效果是一样的；
        在进行下行转换时, dynamic_cast具有类型检查的功能, 比static_cast更安全.
     */
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));

    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        // 将3D前端与3D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } 
  // 2d的轨迹
  else {
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      // local_trajectory_builder(前端)的初始化
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }

    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));

    // CollatedTrajectoryBuilder初始化
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        // 将2D前端与2D位姿图打包在一起, 传入CollatedTrajectoryBuilder
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }

  // 是否是纯定位模式, 如果是则只保存最近3个submap
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  // 如果给了初始位姿
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    
    // 在位姿图中设置初始位姿
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }

  // 保存轨迹的配置文件
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

// 从序列化的数据中构 造一条 trajectory
// 目的
// 为从序列化状态（如 .pbstream 文件）加载的轨迹添加一个占位符。
// 不实际创建轨迹构建器对象，而是将反序列化的轨迹配置信息存储在内部容器中。
// 返回值
// 返回分配给新轨迹的唯一标识符 trajectory_id
int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();

  // c++11: vector::emplace_back() 在原地构造, 直接传入vector, 不调用移动构造函数

  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // 使用 CHECK_EQ 确保 trajectory_builders_ 和 all_trajectory_builder_options_ 的大小一致。
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

// 结束指定id的轨迹, 分别进行 传感器数据处理的结束 与 位姿图的结束
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

// 返回压缩后的地图数据
// 目的
// 从 PoseGraph 中提取指定子地图的压缩数据。
// 将子地图的位姿和数据填充到 response 对象中。
// 返回值
// 如果成功提取子地图数据，返回空字符串。
// 如果子地图不存在或 ID 非法，返回对应的错误信息。
std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  // 进行id的检查
  // 检查 submap_id.trajectory_id 是否合法（范围 [0, num_trajectory_builders() - 1]）。
  // 如果不合法，返回错误信息。
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  // 获取地图数据
  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }

  // 将压缩后的地图数据放入response
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

// 调用 io::WritePbStream 保存所有数据, 没有使用
// 目的
// 将 MapBuilder 的当前状态序列化并写入到指定的 ProtoStreamWriter 中。
// 序列化内容包括：
// 位姿图（PoseGraph）。
// 所有轨迹的构建器配置信息。
// 未完成的子地图（可选）。
void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps);
}

// 将数据进行压缩,并保存到文件中
bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps);
  return (writer.Close());
}

// 从pbstream文件向位姿图添加信息
// 目的
// 从 .pbstream 文件（序列化的 SLAM 状态文件）中加载 SLAM 系统的轨迹、位姿图、传感器数据等，并重建其在内存中的状态。
// 返回值
// 返回一个 std::map<int, int>，表示从 .pbstream 文件中的轨迹 ID 映射到新生成的轨迹 ID。
// reader：
// 类型：io::ProtoStreamReaderInterface* const
// 含义：负责读取 .pbstream 文件中的序列化数据。
// 作用：提供对文件内容的逐步访问能力。
// load_frozen_state：
// 类型：bool
// 含义：
// true：加载后的轨迹设置为冻结状态（不再接收新数据）。
// false：加载后的轨迹处于可更新状态。
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  //使用 reader 创建一个 ProtoStreamDeserializer 对象，负责解析 .pbstream 文件。
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  // 从文件中读取轨迹构建器的选项和位姿图的序列化数据。
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  // key为pbstream文件中的轨迹id, value为新生成的轨迹的id
  // 用于记录文件中的轨迹 ID 和重新生成的轨迹 ID 之间的映射关系。
  std::map<int, int> trajectory_remapping;

  // 从文件中添加轨迹
  // 遍历所有轨迹，为每个轨迹分配新的 ID，并更新位姿图中的轨迹 ID
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    // 添加新轨迹
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    // 原始轨迹id与新生成的轨迹id组成map,放入trajectory_remapping中
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    // 将轨迹id设置为新生成的id
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      // 将指定轨迹id设置为FROZEN状态
      // 冻结轨迹：
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  // 更新约束中节点与子图的轨迹id
  // 目的
  // 更新位姿图中所有约束的轨迹 ID。
  // 在反序列化时，轨迹 ID 会被重新分配，因此需要将 .pbstream 文件中的旧轨迹 ID 映射为新轨迹 ID。
  //遍历约束集合：
  //pose_graph_proto.mutable_constraint() 返回约束集合的可修改指针。
  // 从约束中获取子地图的轨迹 ID：constraint_proto.submap_id().trajectory_id()。
  // 使用 trajectory_remapping 查找新轨迹 ID。
  // 更新子地图的轨迹 ID。
  //从约束中获取节点的轨迹 ID：constraint_proto.node_id().trajectory_id()。
  //使用 trajectory_remapping 查找新轨迹 ID。
  //更新节点的轨迹 ID。
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  // 从获取到的位姿图中生成submap_poses
  // 加载子地图位姿：
  //目的
  //从反序列化的 pose_graph_proto 数据中提取子地图的位姿（submap_pose）。
  //将子地图的 ID 和其位姿存储在 MapById<SubmapId, transform::Rigid3d> 容器中，便于后续处理。
  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  // 从获取到的位姿图中生成node_poses
  // 加载节点位姿：
  // 从反序列化的 pose_graph_proto 数据中提取所有节点的全局位姿。
  // 将节点的唯一标识符（NodeId）和其位姿（transform::Rigid3d）存储在 MapById<NodeId, transform::Rigid3d> 容器中。
  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  // 将landmark_poses添加到位姿图中
  // 为每个地标设置全局位姿
  // 目的
  // 从反序列化的 pose_graph_proto 中提取所有地标的全局位姿信息，并将其设置到位姿图（pose_graph_）中。
  // 作用
  // 地标（Landmark）是 SLAM 系统中特殊的固定点，用于提升定位和约束能力。
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }

  if (options_.use_trajectory_builder_3d()) {
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
  }

  SerializedData proto;
  // 向pose_graph_中添加信息
  // 目的
  // 从序列化数据流中逐条读取数据（SerializedData），并将其添加到 pose_graph_（位姿图）中。
  // 支持多种类型的数据，包括子地图、节点、轨迹数据、传感器数据等。
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        // 为submap设置新的轨迹id
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        // 将submap添加到位姿图中
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }
      case SerializedData::kNode: {
        // 为node_id设置新的轨迹id
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        // 将node_pose添加到位姿图中
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        // 将TrajectoryData添加到位姿图中
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        // 将IMU数据添加到位姿图中
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        // 将Odom数据添加到位姿图中
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        // 将GPS数据添加到位姿图中
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        // 将landmark数据添加到位姿图中
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  // 添加子图的附属的节点
  // 冻结状态（load_frozen_state == true）：只处理子图内部的节点约束。
  // 非冻结状态（load_frozen_state == false）：直接使用序列化的所有约束。
  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      // 如果子图外约束就跳过, 只向子图添加子图内约束的节点
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      // 添加子图的附属的节点
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } 
  else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}

// 从pbstream文件读取信息
std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  // 检查后缀名
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

// 工厂函数
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
