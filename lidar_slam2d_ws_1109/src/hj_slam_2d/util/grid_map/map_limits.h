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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "xy_index.h"

namespace hjSlam_2d
{
    namespace mapping
    {

        // Defines the limits of a grid map. This class must remain inlined for
        // performance reasons.
        class MapLimits
        {
        public:
            MapLimits(const double resolution, const Eigen::Vector2d &max,
                      const CellLimits &cell_limits)
                : resolution_(resolution), max_(max), cell_limits_(cell_limits)
            {
                // CHECK_GT(resolution_, 0.);
                // CHECK_GT(cell_limits.num_x_cells, 0.);
                // CHECK_GT(cell_limits.num_y_cells, 0.);
            }

            // Returns the cell size in meters. All cells are square and the resolution is
            // the length of one side.
            double resolution() const { return resolution_; }

            // Returns the corner of the limits, i.e., all pixels have positions with
            // smaller coordinates.
            const Eigen::Vector2d &max() const { return max_; }

            // Returns the limits of the grid in number of cells.
            const CellLimits &cell_limits() const { return cell_limits_; }

            // Returns the index of the cell containing the 'point' which may be outside
            // the map, i.e., negative or too large indices that will return false for
            // Contains().
            Eigen::Array2i GetCellIndex(const Eigen::Vector2f &point) const
            {
                // Index values are row major and the top left has Eigen::Array2i::Zero()
                // and contains (centered_max_x, centered_max_y). We need to flip and
                // rotate.
                return Eigen::Array2i(
                    std::lround((max_.y() - point.y()) / resolution_ - 0.5),           //取整四舍五入
                    std::lround((max_.x() - point.x()) / resolution_ - 0.5));
            }

            // Returns true if the ProbabilityGrid contains 'cell_index'.
            bool Contains(const Eigen::Array2i &cell_index) const
            {
                return (Eigen::Array2i(0, 0) <= cell_index).all() &&
                       (cell_index <
                        Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
                           .all();
            }

        private:
            double resolution_;
            Eigen::Vector2d max_;
            CellLimits cell_limits_;    //
        };

    } // namespace mapping
} // namespace hjSlam_2d

#endif // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
