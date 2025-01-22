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

#include "pose_extrapolator.h"
#include <algorithm>
#include "../../util/rigid_transform.h"

namespace hjSlam_2d
{
  namespace mapping
  {

    PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                       double imu_gravity_time_constant)
        : pose_queue_duration_(pose_queue_duration),
          gravity_time_constant_(imu_gravity_time_constant),
          cached_extrapolated_pose_{common::Time::min(),
                                    transform::Rigid3d::Identity()} {}

    common::Time PoseExtrapolator::GetLastPoseTime() const
    {
      if (timed_pose_queue_.empty())
      {
        return common::Time::min();
      }
      return timed_pose_queue_.back().time;
    }

    common::Time PoseExtrapolator::GetLastExtrapolatedTime() const
    {
      if (!extrapolation_imu_tracker_)
      {
        return common::Time::min();
      }
      
      return extrapolation_imu_tracker_->time();
    }

    void PoseExtrapolator::addPose(const common::Time time,
                                   const transform::Rigid3d &pose)
    {
      if (imu_tracker_ == nullptr)
      {
        common::Time tracker_start = time;
        if (!imu_data_.empty())
        {
          tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ =
            std::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
      }

      timed_pose_queue_.push_back(TimedPose{time, pose});
      while (timed_pose_queue_.size() > 2 &&
             timed_pose_queue_[1].time <= time - pose_queue_duration_){
        timed_pose_queue_.pop_front();
      }

      updateVelocitiesFromPoses(); // 根据位姿计算当前机器人线速度与角速度
      advanceImuTracker(time, imu_tracker_.get());
      trimImuData();
      trimOdometryData();
      odometry_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);
      extrapolation_imu_tracker_ = std::make_unique<ImuTracker>(*imu_tracker_);

    }

    void PoseExtrapolator::addImuData(const sensor::ImuData &imu_data)
    {
      // CHECK(timed_pose_queue_.empty() ||
      //     imu_data.time >= timed_pose_queue_.back().time);
      imu_data_.push_back(imu_data);

      trimImuData();
    }

    void PoseExtrapolator::AddOdometryData(
        const sensor::OdometryData &odometry_data)
    {

      odometry_data_.push_back(odometry_data);
      trimOdometryData();
      if (odometry_data_.size() < 2)
      {
        return;
      }
      // TODO(whess): Improve by using more than just the last two odometry poses.
      // Compute extrapolation in the tracking frame.
      const sensor::OdometryData &odometry_data_oldest = odometry_data_.front();
      const sensor::OdometryData &odometry_data_newest = odometry_data_.back();
      const double odometry_time_delta =
          common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
      const transform::Rigid3d odometry_pose_delta =
          odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
      angular_velocity_from_odometry_ =
          transform::RotationQuaternionToAngleAxisVector(
              odometry_pose_delta.rotation()) /
          odometry_time_delta;

      if (timed_pose_queue_.empty())
      {
        return;
      }

      //里程计坐标系下的线速度
      const Eigen::Vector3d
          linear_velocity_in_tracking_frame_at_newest_odometry_time =
              odometry_pose_delta.translation() / odometry_time_delta;

      //
      const Eigen::Quaterniond orientation_at_newest_odometry_time =
          timed_pose_queue_.back().pose.rotation() *
          extrapolateRotation(odometry_data_newest.time,
                              odometry_imu_tracker_.get());

      linear_velocity_from_odometry_ =
          orientation_at_newest_odometry_time *
          linear_velocity_in_tracking_frame_at_newest_odometry_time;
    }

    transform::Rigid3d PoseExtrapolator::extrapolatePose(const common::Time time)
    {
      const TimedPose &newest_timed_pose = timed_pose_queue_.back();
      //   CHECK_GE(time, newest_timed_pose.time);
      if (cached_extrapolated_pose_.time != time)
      {
        const Eigen::Vector3d translation =
            extrapolateTranslation(time) + newest_timed_pose.pose.translation();
        const Eigen::Quaterniond rotation =
            newest_timed_pose.pose.rotation() *

            extrapolateRotation(time, extrapolation_imu_tracker_.get());

        cached_extrapolated_pose_ =
            TimedPose{time, transform::Rigid3d{translation, rotation}};
      }
      return cached_extrapolated_pose_.pose;
    }

    Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
        const common::Time time)
    {
      ImuTracker imu_tracker = *imu_tracker_;
      advanceImuTracker(time, &imu_tracker);
      return imu_tracker.orientation();
    }

    void PoseExtrapolator::updateVelocitiesFromPoses()
    {
      if (timed_pose_queue_.size() < 2)
      {
        // We need two poses to estimate velocities.
        return;
      }
      // CHECK(!timed_pose_queue_.empty());
      const TimedPose &newest_timed_pose = timed_pose_queue_.back();
      const auto newest_time = newest_timed_pose.time;
      const TimedPose &oldest_timed_pose = timed_pose_queue_.front();
      const auto oldest_time = oldest_timed_pose.time;
      const double queue_delta = common::ToSeconds(newest_time - oldest_time);
      if (queue_delta < common::ToSeconds(pose_queue_duration_))
      {
        std::cout << "Queue too short for velocity estimation. Queue duration: "
                  << queue_delta << " s" << std::endl;
        return;
      }
      const transform::Rigid3d &newest_pose = newest_timed_pose.pose;
      const transform::Rigid3d &oldest_pose = oldest_timed_pose.pose;
      linear_velocity_from_poses_ =
          (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
      angular_velocity_from_poses_ =
          transform::RotationQuaternionToAngleAxisVector(
              oldest_pose.rotation().inverse() * newest_pose.rotation()) /
          queue_delta;
    }

    void PoseExtrapolator::trimImuData()
    {
      while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
             imu_data_[1].time <= timed_pose_queue_.back().time)
      {
        imu_data_.pop_front();
      }
    }

    void PoseExtrapolator::trimOdometryData()
    {
      while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
             odometry_data_[1].time <= timed_pose_queue_.back().time)
      {
        odometry_data_.pop_front();
      }
    }

    void PoseExtrapolator::advanceImuTracker(const common::Time time,
                                             ImuTracker *const imu_tracker) const
    {
      // CHECK_GE(time, imu_tracker->time());
      if (imu_data_.empty() || time < imu_data_.front().time)
      {
        //std::cout << "------use_LidarPose-----extrapolator" << imu_data_.size()<< std::endl;
        // There is no IMU data until 'time', so we advance the ImuTracker and use
        // the angular velocities from poses and fake gravity to help 2D stability.
        imu_tracker->advance(time);               //根据前一时刻的角速度以及姿态推算推算当前的姿态角度
        imu_tracker->addImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());//(0,0,1)
        imu_tracker->addImuAngularVelocityObservation(
            odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                      : angular_velocity_from_odometry_); //更新当前的角速度
        return;
      }

      if (imu_tracker->time() < imu_data_.front().time)
      {
        // advance to the beginning of 'imu_data_'.
        imu_tracker->advance(imu_data_.front().time);
      }
      auto it = std::lower_bound(
          imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
          [](const sensor::ImuData &imu_data, const common::Time &time)
          {
            return imu_data.time < time;
          });
      while (it != imu_data_.end() && it->time < time)
      {
        // std::cout << "------use_IMU-----extrapolator" << std::endl;
        imu_tracker->advance(it->time);
        imu_tracker->addImuLinearAccelerationObservation(it->linear_acceleration);
        imu_tracker->addImuAngularVelocityObservation(it->angular_velocity);
        ++it;
      }
      imu_tracker->advance(time);
    }

    Eigen::Quaterniond PoseExtrapolator::extrapolateRotation(
        const common::Time time, ImuTracker *const imu_tracker) const
    {
      // CHECK_GE(time, imu_tracker->time());
      advanceImuTracker(time, imu_tracker);
      const Eigen::Quaterniond last_orientation = imu_tracker_->orientation(); // imu_tracker_和imu_tracker的区别？
      return last_orientation.inverse() * imu_tracker->orientation();
    }

    Eigen::Vector3d PoseExtrapolator::extrapolateTranslation(common::Time time)
    {
      const TimedPose &newest_timed_pose = timed_pose_queue_.back();
      const double extrapolation_delta =
          common::ToSeconds(time - newest_timed_pose.time);
      if (odometry_data_.size() < 2)
      {
        return extrapolation_delta * linear_velocity_from_poses_;
      }
      return extrapolation_delta * linear_velocity_from_odometry_;
    }

    PoseExtrapolator::ExtrapolationResult
    PoseExtrapolator::extrapolatePosesWithGravity(
        const std::vector<common::Time> &times)
    {
      std::vector<transform::Rigid3f> poses;
      for (auto it = times.begin(); it != std::prev(times.end()); ++it)
      {
        poses.push_back(extrapolatePose(*it).cast<float>());
      }

      const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                                   ? linear_velocity_from_poses_
                                                   : linear_velocity_from_odometry_;
      return ExtrapolationResult{poses, extrapolatePose(times.back()),
                                 current_velocity,
                                 EstimateGravityOrientation(times.back())};
    }

  } // namespace mapping
} // namespace hjSlam_2d
