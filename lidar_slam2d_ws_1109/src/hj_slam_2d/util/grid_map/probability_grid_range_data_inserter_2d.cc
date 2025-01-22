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

#include "probability_grid_range_data_inserter_2d.h"
#include <cstdlib>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "../../util/ray_to_pixel_mask.h"
#include "probability_values.h"

namespace hjSlam_2d
{
  namespace mapping
  {
    namespace
    {
      // Factor for subpixel accuracy of start and end point for ray casts.
      constexpr int kSubpixelScale = 1000;

      void GrowAsNeeded(const hjSlam_2d::RangeData &range_data,
                        ProbabilityGrid *const probability_grid)
      {
        Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
        // Padding around bounding box to avoid numerical issues at cell boundaries.
        constexpr float kPadding = 1e-6f;
        for (const hjSlam_2d::RangefinderPoint &hit : range_data.returns)
        {
          bounding_box.extend(hit.position.head<2>());
        }
        for (const hjSlam_2d::RangefinderPoint &miss : range_data.misses)
        {
          bounding_box.extend(miss.position.head<2>());
        }
        probability_grid->GrowLimits(bounding_box.min() -
                                     kPadding * Eigen::Vector2f::Ones());
        probability_grid->GrowLimits(bounding_box.max() +
                                     kPadding * Eigen::Vector2f::Ones());
      }

      void CastRays(const hjSlam_2d::RangeData &range_data,
                    const std::vector<uint16> &hit_table,
                    const std::vector<uint16> &miss_table,
                    const bool insert_free_space, ProbabilityGrid *probability_grid)
      {
        GrowAsNeeded(range_data, probability_grid);

        const MapLimits &limits = probability_grid->limits();
        const double superscaled_resolution = limits.resolution() / kSubpixelScale;
        const MapLimits superscaled_limits(
            superscaled_resolution, limits.max(),
            CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                       limits.cell_limits().num_y_cells * kSubpixelScale));
        const Eigen::Array2i begin = superscaled_limits.GetCellIndex(range_data.origin.head<2>());
        // Compute and add the end points.
        std::vector<Eigen::Array2i> ends;
        ends.reserve(range_data.returns.size());
        for (const hjSlam_2d::RangefinderPoint &hit : range_data.returns)
        {
          ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
          probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
          
        }

        if (!insert_free_space)
        {
          return;
        }

        // Now add the misses.
        for (const Eigen::Array2i &end : ends)
        {
          std::vector<Eigen::Array2i> ray =
              RayToPixelMask(begin, end, kSubpixelScale);
          for (const Eigen::Array2i &cell_index : ray)
          {
            probability_grid->ApplyLookupTable(cell_index, miss_table);
          }
        }

        // Finally, compute and add empty rays based on misses in the range data.
        for (const hjSlam_2d::RangefinderPoint &missing_echo : range_data.misses)
        {
          std::vector<Eigen::Array2i> ray = RayToPixelMask(
              begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
              kSubpixelScale);
          for (const Eigen::Array2i &cell_index : ray)
          {
            probability_grid->ApplyLookupTable(cell_index, miss_table);
          }
        }
      }
    } // namespace

    ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(): 
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(0.55))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(0.49))) {}

    void ProbabilityGridRangeDataInserter2D::Insert(
        const hjSlam_2d::RangeData &range_data, Grid2D *const grid) const
    {
      ProbabilityGrid *const probability_grid = static_cast<ProbabilityGrid *>(grid);
      // CHECK(probability_grid != nullptr);
      //  By not finishing the update after hits are inserted, we give hits priority
      //  (i.e. no hits will be ignored because of a miss in the same cell).
      CastRays(range_data, hit_table_, miss_table_, /*options_.insert_free_space()*/ true, probability_grid);
      probability_grid->FinishUpdate();
    }

  } // namespace mapping
} // namespace hjSlam_2d
