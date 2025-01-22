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

#include "motion_filter.h"
// #include "glog/logging.h"

namespace hjSlam_2d
{
    namespace mapping
    {

        MotionFilter::MotionFilter(){}

        bool MotionFilter::IsSimilar(const common::Time time,
                                     const transform::Rigid3d &pose)
        {

            //   LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
            //       << "Motion filter reduced the number of nodes to "
            //       << 100. * num_different_ / num_total_ << "%.";
            ++num_total_;
            if (num_total_ > 1 &&
                time - last_time_ <= common::FromSeconds(5.0) &&
                (pose.translation() - last_pose_.translation()).norm() <=
                    0.2 &&
                transform::GetAngle(pose.inverse() * last_pose_) <=
                    0.017)
            {
                return true;
            }
            last_time_ = time;
            last_pose_ = pose;
            ++num_different_;
            return false;
        }

    } // namespace mapping
} // namespace hjSlam_2d
