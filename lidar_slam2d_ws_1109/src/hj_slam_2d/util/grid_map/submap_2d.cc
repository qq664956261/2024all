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

#include "submap_2d.h"
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "memory.h"
#include "../../util/port.h"

namespace hjSlam_2d
{
    namespace mapping
    {

        Submap2D::Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
                           ValueConversionTables *conversion_tables):conversion_tables_(conversion_tables)
        {
            grid_ = std::move(grid);
        }

        void Submap2D::InsertRangeData(
            const hjSlam_2d::RangeData &range_data,
            const ProbabilityGridRangeDataInserter2D *range_data_inserter)
        {
            // CHECK(grid_);
            // CHECK(!insertion_finished());
            range_data_inserter->Insert(range_data, grid_.get());
            set_num_range_data(num_range_data() + 1);
        }

        void Submap2D::Finish()
        {
            // CHECK(grid_);
            // CHECK(!insertion_finished());
            grid_ = grid_->ComputeCroppedGrid();
            set_insertion_finished(true);
        }

        ActiveSubmaps2D::ActiveSubmaps2D(/*const proto::SubmapsOptions2D &options*/)
            : /*options_(options), */ range_data_inserter_(CreateRangeDataInserter()) {}

        std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const
        {
            return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(), submaps_.end());
        }

        std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
            const hjSlam_2d::RangeData &range_data)
        {
            if (submaps_.empty() ||
                submaps_.back()->num_range_data() == 9000000 /*options_.num_range_data()*/)
            {
                AddSubmap(range_data.origin.head<2>());
            }

            for (auto &submap : submaps_)
            {
                submap->InsertRangeData(range_data, range_data_inserter_.get());
            }

            if (submaps_.front()->num_range_data() == 2 * 900000 /*options_.num_range_data()*/)
            {
                submaps_.front()->Finish();
            }

            return submaps();
        }

        std::unique_ptr<ProbabilityGridRangeDataInserter2D>
        ActiveSubmaps2D::CreateRangeDataInserter()
        {
            return std::make_unique<ProbabilityGridRangeDataInserter2D>(
                /*options_.range_data_inserter_options()
                    .probability_grid_range_data_inserter_options_2d()*/
            );
            // switch (options_.range_data_inserter_options().range_data_inserter_type())
            // {
            // case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
            //     return std::make_unique<ProbabilityGridRangeDataInserter2D>(
            //         options_.range_data_inserter_options()
            //             .probability_grid_range_data_inserter_options_2d());
            // case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
            //     return absl::make_unique<TSDFRangeDataInserter2D>(
            //         options_.range_data_inserter_options()
            //             .tsdf_range_data_inserter_options_2d());
            // default:
            //     LOG(FATAL) << "Unknown RangeDataInserterType.";
            // }
        }

        std::unique_ptr<Grid2D> ActiveSubmaps2D::CreateGrid(
            const Eigen::Vector2f &origin)
        {
            constexpr int kInitialSubmapSize = 100;
            float resolution = /*options_.grid_options_2d().resolution()*/ 0.05;

            return std::make_unique<ProbabilityGrid>(
                MapLimits(resolution,
                          origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                      resolution *
                                                      Eigen::Vector2d::Ones(),
                          CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
                &conversion_tables_);
        }

        void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f &origin)
        {
            if (submaps_.size() >= 2)
            {
                // This will crop the finished Submap before inserting a new Submap to
                // reduce peak memory usage a bit.
                // CHECK(submaps_.front()->insertion_finished());
                submaps_.erase(submaps_.begin());
            }
            submaps_.push_back(std::make_unique<Submap2D>(
                origin,
                std::unique_ptr<Grid2D>(static_cast<Grid2D *>(CreateGrid(origin).release())),
                &conversion_tables_));
        }

    } // namespace mapping
} // namespace hjSlam_2d
