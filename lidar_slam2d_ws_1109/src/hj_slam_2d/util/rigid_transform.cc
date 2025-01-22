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

#include "rigid_transform.h"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace hjSlam_2d {
namespace transform {

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle; // 先绕Z轴旋转，其次绕Y轴旋转，最后绕X轴旋转
}

Rigid3d eigen_Matrix4dToRigid3d(const Eigen::Matrix4d transformation)
{
  Eigen::Quaterniond qr(transformation.block<3, 3>(0, 0));
  qr.normalize();
  Eigen::Vector3d t = transformation.block<3, 1>(0, 3);
  return Rigid3d({t.x(), t.y(), t.z()}, qr);
}

}  // namespace transform
}  // namespace hjSlam_2d
