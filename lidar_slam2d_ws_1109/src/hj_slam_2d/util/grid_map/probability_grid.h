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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "../../util/port.h"
#include "grid_2d.h"
#include "map_limits.h"
#include "xy_index.h"
#include <map>

namespace hjSlam_2d
{
  namespace mapping
  {
    // Represents a 2D grid of probabilities.
    class ProbabilityGrid : public Grid2D
    {
    public:
      explicit ProbabilityGrid(const MapLimits &limits,
                               ValueConversionTables *conversion_tables);

      // Sets the probability of the cell at 'cell_index' to the given
      // 'probability'. Only allowed if the cell was unknown before.
      void SetProbability(const Eigen::Array2i &cell_index,
                          const float probability);

      // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
      // to the probability of the cell at 'cell_index' if the cell has not already
      // been updated. Multiple updates of the same cell will be ignored until
      // FinishUpdate() is called. Returns true if the cell was updated.
      //
      // If this is the first call to ApplyOdds() for the specified cell, its value
      // will be set to probability corresponding to 'odds'.
      bool ApplyLookupTable(const Eigen::Array2i &cell_index,
                            const std::vector<uint16> &table);

      GridType GetGridType() const override;

      // Returns the probability of the cell with 'cell_index'.
      float GetProbability(const Eigen::Array2i &cell_index) const;

      std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;

    private:
      ValueConversionTables *conversion_tables_;
    };

  } // namespace mapping
} // namespace hjSlam_2d

#endif // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
