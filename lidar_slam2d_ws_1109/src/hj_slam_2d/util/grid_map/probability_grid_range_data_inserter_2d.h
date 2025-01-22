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

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>
#include "../../util/point_cloud.h"
#include "../../util/port.h"
#include "probability_grid.h"
#include "xy_index.h"

namespace hjSlam_2d
{
    namespace mapping
    {

        class ProbabilityGridRangeDataInserter2D
        {
        public:
            explicit ProbabilityGridRangeDataInserter2D();
            ~ProbabilityGridRangeDataInserter2D(){};

            ProbabilityGridRangeDataInserter2D(
                const ProbabilityGridRangeDataInserter2D &) = delete;
            ProbabilityGridRangeDataInserter2D &operator=(
                const ProbabilityGridRangeDataInserter2D &) = delete;

            // Inserts 'range_data' into 'probability_grid'.
            void Insert(const hjSlam_2d::RangeData &range_data,
                                Grid2D *grid) const;

        private:
            //const proto::ProbabilityGridRangeDataInserterOptions2D options_;
            const std::vector<uint16> hit_table_;
            const std::vector<uint16> miss_table_;
        };

    } // namespace mapping
} // namespace hjSlam_2d

#endif // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
