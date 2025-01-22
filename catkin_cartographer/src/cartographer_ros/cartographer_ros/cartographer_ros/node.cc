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

#include "cartographer_ros/node.h"

#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "geometry_msgs/PoseStamped.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

// lx add
#include <pcl/io/pcd_io.h>

#include "cartographer/mapping/id.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

// lx add
using ::cartographer::mapping::MapById;
using ::cartographer::mapping::NodeId;
using ::cartographer::mapping::TrajectoryNode;
using ::cartographer::sensor::RangefinderPoint;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.

/**
 * @brief 在node_handle中订阅topic,并与传入的回调函数进行注册
 *
 * @tparam MessageType 模板参数,消息的数据类型
 * @param[in] handler 函数指针, 接受传入的函数的地址
 * @param[in] trajectory_id 轨迹id
 * @param[in] topic 订阅的topic名字
 * @param[in] node_handle ros的node_handle
 * @param[in] node node类的指针
 * @return ::ros::Subscriber 订阅者
 */
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(
        int, const std::string&,
        const typename MessageType::
            ConstPtr&),  // 这是一个类成员函数指针，指向Node类中用于处理特定类型消息的成员函数。这个函数需要三个参数：轨迹ID、话题名称和消息本身。
    const int trajectory_id, const std::string& topic,  //: 轨迹ID，用于区分来自不同数据源或任务的消息，
                                                        //: 要订阅的话题名称。
    ::ros::NodeHandle* const node_handle,
    Node* const node) {  //::ros::NodeHandle* const node_handle:
                         //:指向节点处理对象的指针。Node* const node:
                         //:指向Node类实例的指针
  // 函数通过node_handle->subscribe<MessageType>调用创建了一个订阅者。定义了两个参数：
  //  topic为要订阅的话题名称，kInfiniteSubscriberQueueSize表示订阅队列的大小。这里kInfiniteSubscriberQueueSize被设置为0，表示订阅队列大小无限
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,  // kInfiniteSubscriberQueueSize = 0
      // 使用boost::function构造回调函数,被subscribe注册
      boost::function<void(const typename MessageType::ConstPtr&)>(
          // c++11: lambda表达式
          // lambda表达式来捕获外部变量node、handler、trajectory_id和topic，并生成回调函数。
          // 当消息msg到达时，会调用Node类的实例node的handler成员函数，并传递trajectory_id、topic以及消息本身
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

// 返回轨迹的状态
std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace

/**
 * @brief
 * 声明ROS的一些topic的发布器, 服务的发布器,
以及将时间驱动的函数与定时器进行绑定
 * node_options: 包含了节点配置的选项。
map_builder: 用于构建SLAM地图的接口，提供前端和后端的功能。
tf_buffer: 用于存储和管理坐标系变换的缓冲区。
collect_metrics: 一个布尔值标志，指示是否收集指标数据。
 *
 * @param[in] node_options 配置文件的内容
 * @param[in] map_builder SLAM算法的具体实现
 * @param[in] tf_buffer tf
 * @param[in] collect_metrics 是否启用metrics,默认不启用
 */
Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {
  // 将mutex_上锁, 防止在初始化时数据被更改
  absl::MutexLock lock(&mutex_);

  // 默认不启用
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  // Step: 1 声明需要发布的topic

  // 发布SubmapList
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  // 发布轨迹
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  // 发布landmark_pose
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  // 发布约束
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  // 发布tracked_pose, 默认不发布
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseStamped>(
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  // lx add
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    point_cloud_map_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            kPointCloudMapTopic, kLatestOnlyPublisherQueueSize, true);
  }

  // Step: 2 声明发布对应名字的ROS服务, 并将服务的发布器放入到vector容器中
  // 处理子地图查询
  // 启动轨迹
  // 完成轨迹
  // 写入状态数据
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));

  // Step: 3 处理之后的点云的发布器
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  // Step: 4 进行定时器与函数的绑定, 定时发布数据
  // std::vector<ros::WallTimer> 类型的容器，用于存储定时器对象。这个容器中的定时器对象用来定期执行任务。这些任务通常是发布某些类型的数据或执行周期性的更新
//   ROS 提供了两种类型的计时器：
//   ros::Timer: 它依赖于ROS系统时间，可以在模拟的时间流逝中被暂停或快进。
// r os::WallTimer: 它依赖于真实的墙上时间（即系统时间），不受ROS时间控制，即使ROS时间被暂停或被操纵，它也会依照现实中的时间流逝。
//   这里使用的是 ros::WallTimer，因为它不受仿真时间的影响，无论ROS时间如何变化，它都会以实际时间流逝。
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),  // 0.3s
      &Node::PublishSubmapList, this));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),  // 5e-3s
        &Node::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(
          node_options_.trajectory_publish_period_sec),  // 30e-3s
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(
          node_options_.trajectory_publish_period_sec),  // 30e-3s
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),  // 0.5s
      &Node::PublishConstraintList, this));
  // lx add
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(kPointCloudMapPublishPeriodSec),  // 10s
        &Node::PublishPointCloudMap, this));
  }
}

// 在析构是执行一次全局优化
Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

/**
 * @brief 获取对应id轨迹的 索引为submap_index 的submap
 *
 * @param[in] request 获取submap的请求
 * @param[out] response 服务的回应
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

/**
 * @brief 获取对应id的轨迹
 *
 * @param[in] request
 * @param[out] response
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);

  // 检查对应id的轨迹是否存在, 如果存在判断一下该id轨迹的状态
  response.status = TrajectoryStateToStatus(
      request.trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  // 获取轨迹
  map_builder_bridge_.HandleTrajectoryQuery(request, response);
  return true;
}

/**
 * @brief 每0.3s发布一次submap list,
 * 这里的submap只有节点的id与当前submap的节点数, 并没有地图数据
 *
 * @param[in] unused_timer_event
 */
void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

/**
 * @brief 新增一个位姿估计器
 *
 * @param[in] trajectory_id 轨迹id
 * @param[in] options 参数配置
 */
// 函数是用于给特定轨迹 (trajectory_id) 添加一个姿态外插器 (PoseExtrapolator) 的。
// 姿态外插器基于过去观测到的位姿和速度来预测（外插）机器人在未来或过去任何时间点的位姿。
// 这在SLAM算法中尤其重要，用于将来自传感器的观测与机器人的预测轨迹对齐。
void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms

  // 新生成的轨迹的id 不应该在extrapolators_中
  // 检查传入的轨迹ID：通过CHECK断言确认trajectory_id不应该已经存在于extrapolators_中。这是因为每个轨迹的位姿外插都应该是唯一的。
  CHECK(extrapolators_.count(trajectory_id) == 0);

  // imu_gravity_time_constant在2d, 3d中都是10
  // 获取重力时间常数：根据是否使用了3D轨迹构造器来选择合适的重力时间常数 (gravity_time_constant)。
  // 这个常数是用来调整位姿外插器内部对IMU数据的处理方式，以反映真实世界中重力加速度对移动对象加速度计算的影响。
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();

  // c++11: map::emplace() 用于通过在容器中插入新元素来扩展map容器
  // 元素是直接构建的（既不复制也不移动）.仅当键不存在时才进行插入
  // c++11: std::forward_as_tuple tuple的完美转发
  // 该 tuple 在以右值为参数时拥有右值引用数据成员, 否则拥有左值引用数据成员
  // c++11: std::piecewise_construct 分次生成tuple的标志常量
  // 在map::emplace()中使用forward_as_tuple时必须要加piecewise_construct,不加就报错
  // https://www.cnblogs.com/guxuanqing/p/11396511.html

  // 以1ms, 以及重力常数10, 作为参数构造PoseExtrapolator
  // 构造位姿外插器：使用一毫秒（kExtrapolationEstimationTimeSec = 0.001）作为估计时间和上一步得到的重力时间常数作为参数，
  // 通过调用extrapolators_.emplace来构造并添加一个新的位姿外插器到extrapolators_容器中。
  // 这里使用std::piecewise_construct和std::forward_as_tuple来处理构造器参数的正确传递，确保位姿外插器能够被正确且高效地构造。
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

/**
 * @brief 新生成一个传感器数据采样器
 *
 * @param[in] trajectory_id 轨迹id
 * @param[in] options 参数配置
 */
// 函数的目的是为指定轨迹 (trajectory_id) 添加传感器数据采样器。这一过程基于给定的轨迹选项 (TrajectoryOptions) 中关于不同传感器数据流的采样比率进行配置。
// 传感器数据采样器用于决定哪些传感器数据会被用于SLAM处理，哪些会被抛弃，以控制数据流的密度和处理负载。
void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  // 使用CHECK断言确认指定轨迹ID (trajectory_id) 尚未在 sensor_samplers_ 容器中。这是为了确保不会重复为同一轨迹添加采样器。
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

/**
 * @brief 每5e-3s发布一次tf与tracked_pose
 *
 * @param[in] timer_event
 */
void Node::PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event) {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetLocalTrajectoryData()) {
    // entry的数据类型为std::unordered_map<int,MapBuilderBridge::LocalTrajectoryData>
    // entry.first 就是轨迹的id, entry.second 就是 LocalTrajectoryData

    // 获取对应轨迹id的trajectory_data
    const auto& trajectory_data = entry.second;

    // 获取对应轨迹id的extrapolator
    auto& extrapolator = extrapolators_.at(entry.first);

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    // 如果当前状态的时间与extrapolator的lastposetime不等
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      // 有订阅才发布scan_matched_point_cloud
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());

        // 获取local_slam_data的点云数据, 填入到point_cloud中
        for (const cartographer::sensor::RangefinderPoint point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          // 这里的虽然使用的是带时间戳的点云结构, 但是数据点的时间全是0.f
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }

        // 先将点云转换成ROS的格式,再发布scan_matched_point_cloud点云
        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            // 将雷达坐标系下的点云转换成地图坐标系下的点云
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }  // end 发布scan_matched_point_cloud

      // 将当前的pose加入到extrapolator中, 更新extrapolator的时间与状态
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }  // end if

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.

    // 使用较新的时间戳
    const ::cartographer::common::Time now = std::max(
        FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? ToRos(now)
            : ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] ==
            stamped_transform.header.stamp)
      continue;

    // 保存当前的时间戳, 以防止对同一时间戳进行重复更新
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;

    // 获取当前位姿在local坐标系下的坐标
    const Rigid3d tracking_to_local = [&] {
      // 是否将变换投影到平面上
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    // 求得当前位姿在map下的坐标
    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;

    // 根据lua配置文件发布tf
    if (trajectory_data.published_to_tracking != nullptr) {
      if (node_options_.publish_to_tf) {
        // 如果需要cartographer提供odom坐标系
        // 则发布 map_frame -> odom -> published_frame 的tf
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::TransformStamped> stamped_transforms;

          // map_frame -> odom
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          // 将local坐标系作为odom坐标系
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform);

          // odom -> published_frame
          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          // published_to_tracking 是局部坐标系下的位姿
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);

          // 发布 map_frame -> odom -> published_frame 的tf
          tf_broadcaster_.sendTransform(stamped_transforms);
        }
        // cartographer不需要提供odom坐标系,则发布 map_frame -> published_frame
        // 的tf
        else {
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          // 发布 map_frame -> published_frame 的tf
          tf_broadcaster_.sendTransform(stamped_transform);
        }
      }

      // publish_tracked_pose 默认为false, 默认不发布
      // 如果设置为true, 就发布一个在tracking_frame处的pose
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
        tracked_pose_publisher_.publish(pose_msg);
      }
    }
  }
}

// 每30e-3s发布一次轨迹路径点数据
void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  // 只有存在订阅者的时候才发布轨迹
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

// 每30e-3s发布一次landmark pose 数据
void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

// 每0.5s发布一次约束数据
void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

/**
 * @brief 根据配置文件, 确定所有需要的topic的名字的集合
 *
 * @param[in] options TrajectoryOptions的配置文件
 * @return std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
 */
// 函数的作用是创建和返回一组用于SLAM进程的传感器标识符，这些标识符指定了Node类需要订阅的ROS主题。
// 该函数根据传入的 TrajectoryOptions 配置来确定SLAM算法需要哪些传感器输入。
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  /*
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    struct SensorId {
      SensorType type;  // 传感器的种类
      std::string id;   // topic的名字
    };
  */

  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  // 初始化传感器ID集合：定义了一个存储 SensorId 结构体的集合 expected_topics 。
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.

  // 如果只有一个传感器, 那订阅的topic就是topic
  // 如果是多个传感器, 那订阅的topic就是topic_1,topic_2, 依次类推
  // 通过遍历激光扫描数据的配置，并为每个数据源生成预期的话题名称，然后将它们添加到 expected_topics 集合中。
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 3d slam必须有imu, 2d可有可无, imu的topic的个数只能有一个
  // 添加IMU数据：如果是3D SLAM，IMU是必需的；如果是2D SLAM且配置了使用IMU数据，那么也会添加IMU数据的Topic。
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  // 里程计可有可无, topic的个数只能有一个
  // 添加里程计数据：如果配置了使用里程计数据，该Topic也会添加到集合中
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  // gps可有可无, topic的个数只能有一个
  // 添加卫星导航数据：如果配置了使用NavSatFix数据（比如GPS），其Topic也会被加入。
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  // Landmark可有可无, topic的个数只能有一个
  // 添加地标数据：如果配置了使用Landmark数据，那么相关Topic名称也会被插入。
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  // 返回传感器的topic名字
  return expected_topics;
}

/**
 * @brief 添加一个新的轨迹
 *
 * @param[in] options 轨迹的参数配置
 * @return int 新生成的轨迹的id
 */
int Node::AddTrajectory(const TrajectoryOptions& options) {
  // 计算期望的传感器ID集合 (ComputeExpectedSensorIds): 根据传入的轨迹选项 options，计算出预期的传感器ID集。这个集合定义了将要使用的所有传感器的类型和命名，以确保SLAM系统能正确响应
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);

  // 调用map_builder_bridge的AddTrajectory, 添加一个轨迹
  // 添加轨迹 (map_builder_bridge_.AddTrajectory): 调用 MapBuilderBridge 的 AddTrajectory 方法以添加一个新的轨迹。
  // 这个新轨迹将用于处理传入的SLAM数据。方法返回值为新轨迹的ID。
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);

  // 新增一个位姿估计器
  // 位姿估计器的添加 (AddExtrapolator): 对于新轨迹，创建一个用于姿态外插的对象。位姿外插允许系统在没有最新观测数据时，预测最新的机器人路经。
  AddExtrapolator(trajectory_id, options);

  // 新生成一个传感器数据采样器
  AddSensorSamplers(trajectory_id, options);

  // 订阅话题与注册回调函数
  LaunchSubscribers(options, trajectory_id);

  // 创建了一个3s执行一次的定时器,由于oneshot=true, 所以只执行一次
  // 检查设置的topic名字是否在ros中存在, 不存在则报错
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(
          kTopicMismatchCheckDelaySec),  // kTopicMismatchCheckDelaySec = 3s
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));

  // 将topic名字保存下来,用于之后的新建轨迹时检查topic名字是否重复
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }

  return trajectory_id;
}

/**
 * @brief 订阅话题与注册回调函数
 *
 * @param[in] options 配置参数
 * @param[in] trajectory_id 轨迹id
 */
//  函数的作用是为 Node 类实例启动一系列 ROS 话题订阅者，每个订阅者绑定到相应的处理函数上，用来处理对应话题上的消息
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  // laser_scan 的订阅与注册回调函数, 多个laser_scan 的topic 共用同一个回调函数
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }

  // multi_echo_laser_scans的订阅与注册回调函数
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // point_clouds 的订阅与注册回调函数
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // imu 的订阅与注册回调函数,只有一个imu的topic
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                &node_handle_, this),
         kImuTopic});
  }

  // odometry 的订阅与注册回调函数,只有一个odometry的topic
  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  &node_handle_, this),
         kOdometryTopic});
  }

  // gps 的订阅与注册回调函数,只有一个gps的topic
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             &node_handle_, this),
         kNavSatFixTopic});
  }

  // landmarks 的订阅与注册回调函数,只有一个landmarks的topic
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             &node_handle_, this),
         kLandmarkTopic});
  }
}

// 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

// 检查topic名字是否被其他轨迹使用
bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    // 如果topic能够在subscribed_topics_中找到,
    // 证明这个topic名字被之前的轨迹使用了
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

/**
 * @brief 检查对应id的轨迹是否存在, 如果存在则判断这个轨迹的状态
 *
 * @param[in] trajectory_id 轨迹的id
 * @param[in] valid_states 状态的列表
 * @return cartographer_ros_msgs::StatusResponse
 */
cartographer_ros_msgs::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  // 如果没有找到对应id的轨迹, 返回NOT_FOUND
  if (it == trajectory_states.end()) {
    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message =
      absl::StrCat("Trajectory ", trajectory_id, " is in '",
                   TrajectoryStateToString(it->second), "' state.");
  // 轨迹的状态如果在列表中, 返回OK, 否则返回 INVALID_ARGUMENT
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::StatusCode::OK
          : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

/**
 * @brief 结束指定id的轨迹
 *
 * @param[in] trajectory_id 要结束的轨迹的id
 * @return cartographer_ros_msgs::StatusResponse
 */
cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;
  // Step: 1 检查 trajectory_id 是否在 正在结束的轨迹集合中
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  // Step: 2 检查这个轨迹是否存在, 如果存在则检查这个轨迹是否是ACTIVE状态
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  // 如果不是OK状态就返回ERROR
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  // Step: 3 如果这个轨迹存在subscribers, 则先关闭subscriber
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    // 在subscribers_中将这条轨迹的信息删除
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }

  // Step: 4 调用cartographer中的map_builder的FinishTrajectory()进行轨迹的结束
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  // 将这个轨迹id放进正在结束的轨迹集合中
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

/**
 * @brief 通过服务来开始一条新的轨迹
 *
 * @param[in] request
 * 配置文件的目录与名字, 是否使用初始位姿, 初始位姿以及其是相对于哪条轨迹的id,
 * @param[out] response 返回轨迹的状态与id
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
// 初始化并开始一个新的轨迹，即启动 SLAM 系统的地图构建过程。
// request: 包含了服务请求中的参数，如配置文件的路径，初始位姿信息，以及请求对应的轨迹ID等。
// response: 是服务的响应，用于返回操作结果状态和新轨迹的ID。
bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  TrajectoryOptions trajectory_options;
  // 获取配置文件内容
  // 获取配置: 从请求中指定的配置文件加载轨迹选项，忽略配置文件的第一部分（使用 std::ignore），因为这里我们只需要轨迹选项。
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request.configuration_directory, request.configuration_basename);

  // 如果给定了一个初始位姿
  // 处理初始位姿: 如果请求包含初始位姿 (use_initial_pose)，将位姿从 ROS 消息转换为 Cartographer 的 Rigid3d 格式，
  // 并检查其有效性。如果位姿无效，则设置响应状态为错误信息后返回。
  if (request.use_initial_pose) {
    const auto pose = ToRigid3d(request.initial_pose);
    if (!pose.IsValid()) {
      response.status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response.status.message;
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    // 检查 initial_pose 对应的轨迹id是否存在
    // 处理轨迹ID: 检查给定的相对初始位姿的轨迹ID是否有效，即请求的轨迹ID是否是当前存在且有效的。
    response.status = TrajectoryStateToStatus(
        request.relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response.status.message;
      return true;
    }
    // 设置初始位姿: 如果初始位姿有效，创建 InitialTrajectoryPose 消息，设置轨迹ID，位姿，以及时间戳。
    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request.relative_to_trajectory_id);
    // 将pose转成proto格式,放进initial_trajectory_pose
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(ros::Time(0))));

    // 将初始位姿信息加入到trajectory_options中
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  // 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
  // 校验轨迹选项: 调用 ValidateTrajectoryOptions 方法检查轨迹选项是否有效，比如确保指定了SLAM模式为2D或3D。
  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response.status.message = "Invalid trajectory options.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  }
  // 检查topic名字是否被其他轨迹使用
  // 校验主题名称: 调用 ValidateTopicNames 方法确保启动的轨迹没有使用已被其他轨迹使用的主题名称
  else if (!ValidateTopicNames(trajectory_options)) {
    response.status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  }
  // 检查通过, 添加一个新的轨迹
  // 添加轨迹: 如果所有的校验都通过了，创建并启动新的轨迹，返回新轨迹的ID和成功的状态码。
  else {
    response.status.message = "Success.";
    response.trajectory_id = AddTrajectory(trajectory_options);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
  }
  return true;
}

// 使用默认topic名字开始一条轨迹,也就是开始slam
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  // 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
  CHECK(ValidateTrajectoryOptions(options));
  // 添加一条轨迹
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

/**
 * @brief 获取所有轨迹的状态
 *
 * @param[in] request 无
 * @param[out] response 返回所有轨迹的状态
 * @return true
 */
bool Node::HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response) {
  // enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

  absl::MutexLock lock(&mutex_);
  response.status.code = ::cartographer_ros_msgs::StatusCode::OK;
  response.trajectory_states.header.stamp = ros::Time::now();
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    // 轨迹的id
    response.trajectory_states.trajectory_id.push_back(entry.first);
    // 每个轨迹对应一个TrajectoryStates
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

/**
 * @brief 结束一条轨迹
 *
 * @param[in] request 轨迹的id
 * @param[out]] response 返回StatusResponse格式的处理状态
 * @return true
 */
bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

/**
 * @brief
 * 当前状态序列化为proto流文件.如果将'include_unfinished_submaps'设置为true,
 * 则未完成的子图（即尚未接收到所有测距仪数据插入的子图）将包含在序列化状态中.
 *
 * @param[in] request 要生成的文件名,以及是否包含未完成的submap
 * @param[out] response
 * @return true
 */
bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  {
    absl::MutexLock lock(&mutex_);
    // 直接调用cartographer的map_builder_的SerializeStateToFile()函数进行文件的保存
    if (map_builder_bridge_.SerializeState(
            request.filename, request.include_unfinished_submaps)) {
      response.status.code = cartographer_ros_msgs::StatusCode::OK;
      response.status.message =
          absl::StrCat("State written to '", request.filename, "'.");
    } else {
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      response.status.message =
          absl::StrCat("Failed to write '", request.filename, "'.");
    }
  }
  // lx add
  constexpr bool save_pcd = false;
  if (node_options_.map_builder_options.use_trajectory_builder_3d() &&
      save_pcd) {
    absl::MutexLock lock(&point_cloud_map_mutex_);
    const std::string suffix = ".pbstream";
    std::string prefix =
        request.filename.substr(0, request.filename.size() - suffix.size());

    LOG(INFO) << "Saving map to pcd files ...";
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_map(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(ros_point_cloud_map_, *pcl_point_cloud_map);
    pcl::io::savePCDFileASCII(prefix + ".pcd", *pcl_point_cloud_map);
    LOG(INFO) << "Pcd written to " << prefix << ".pcd";
  }
  return true;
}

/**
 * @brief 读取metrics
 *
 * @param[in] request 无
 * @param[out] response
 * @return true
 */
// Node::HandleReadMetrics 函数设计用于ROS服务请求，目的是获取运行时的监测指标（即metrics）。
// 当其他节点通过ROS服务调用这个函数时，它会收集并返回Cartographer SLAM系统当前的性能和状态指标
bool Node::HandleReadMetrics(
    ::cartographer_ros_msgs::ReadMetrics::Request& request,
    ::cartographer_ros_msgs::ReadMetrics::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.timestamp = ros::Time::now();
  if (!metrics_registry_) {
    response.status.code = cartographer_ros_msgs::StatusCode::UNAVAILABLE;
    response.status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(&response);
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = "Successfully read metrics.";
  return true;
}

// 结束所有处于活动状态的轨迹
void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

// 结束指定id的轨迹
bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

// 当所有的轨迹结束时, 执行一次全局优化
void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

/**
 * @brief 处理里程计数据,里程计的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测
 * 第2个是传入SensorBridge,使用其传感器处理函数进行里程计数据处理
 *
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id 里程计的topic名字
 * @param[in] msg 里程计的ros格式的数据
 */
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

/**
 * @brief 处理imu数据,imu的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测与重力方向的确定
 * 第2个是传入SensorBridge,使用其传感器处理函数进行imu数据处理
 *
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id imu的topic名字
 * @param[in] msg imu的ros格式的数据
 */
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  // 根据配置,是否将传感器数据跳过
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

// 加载pbstream文件
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
  load_state_ = true;
}

// 检查设置的topic名字是否在ros中存在, 不存在则报错
void Node::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& unused_timer_event) {
  // note: 使用ros的master的api进行topic名字的获取
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);

  std::set<std::string> published_topics;
  std::stringstream published_topics_string;

  // 获取ros中的实际topic的全局名称,resolveName()是获取全局名称
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }

  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      // 获取实际订阅的topic名字
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);

      // 如果设置的topic名字,在ros中不存在,则报错
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  // 告诉使用者哪些topic可用
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}

void Node::PublishPointCloudMap(const ::ros::WallTimerEvent& timer_event) {
  // 纯定位时不发布点云地图
  if (load_state_ || point_cloud_map_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // 只发布轨迹id 0 的点云地图
  constexpr int trajectory_id = 0;

  // 获取优化后的节点位姿与节点的点云数据
  std::shared_ptr<MapById<NodeId, TrajectoryNode>> trajectory_nodes =
      map_builder_bridge_.GetTrajectoryNodes();

  // 如果个数没变就不进行地图发布
  size_t trajectory_nodes_size =
      trajectory_nodes->SizeOfTrajectoryOrZero(trajectory_id);
  if (last_trajectory_nodes_size_ == trajectory_nodes_size) return;
  last_trajectory_nodes_size_ = trajectory_nodes_size;

  absl::MutexLock lock(&point_cloud_map_mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr node_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  // 遍历轨迹0的所有优化后的节点
  auto node_it = trajectory_nodes->BeginOfTrajectory(trajectory_id);
  auto end_it = trajectory_nodes->EndOfTrajectory(trajectory_id);
  for (; node_it != end_it; ++node_it) {
    auto& trajectory_node = trajectory_nodes->at(node_it->id);
    auto& high_resolution_point_cloud =
        trajectory_node.constant_data->high_resolution_point_cloud;
    auto& global_pose = trajectory_node.global_pose;

    if (trajectory_node.constant_data != nullptr) {
      node_point_cloud->clear();
      node_point_cloud->resize(high_resolution_point_cloud.size());
      // 遍历点云的每一个点, 进行坐标变换
      for (const RangefinderPoint& point :
           high_resolution_point_cloud.points()) {
        RangefinderPoint range_finder_point = global_pose.cast<float>() * point;
        node_point_cloud->push_back(pcl::PointXYZ(
            range_finder_point.position.x(), range_finder_point.position.y(),
            range_finder_point.position.z()));
      }
      // 将每个节点的点云组合在一起
      *point_cloud_map += *node_point_cloud;
    }
  }  // end for

  ros_point_cloud_map_.data.clear();
  pcl::toROSMsg(*point_cloud_map, ros_point_cloud_map_);
  ros_point_cloud_map_.header.stamp = ros::Time::now();
  ros_point_cloud_map_.header.frame_id = node_options_.map_frame;
  LOG(INFO) << "publish point cloud map";
  point_cloud_map_publisher_.publish(ros_point_cloud_map_);
}

}  // namespace cartographer_ros