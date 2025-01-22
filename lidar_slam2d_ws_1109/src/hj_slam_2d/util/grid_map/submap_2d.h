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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "grid_2d.h"
#include "map_limits.h"
#include "../../util/point_cloud.h"
#include "../../util/rigid_transform.h"
#include "probability_grid_range_data_inserter_2d.h"

namespace hjSlam_2d
{
  namespace mapping
  {
    // Converts the given probability to log odds.
    inline float Logit(float probability)
    {
      return std::log(probability / (1.f - probability));
    }

    const float kMaxLogOdds = Logit(kMaxProbability);
    const float kMinLogOdds = Logit(kMinProbability);

    // Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
    // kMaxLogOdds] is mapped to [1, 255].
    inline uint8 ProbabilityToLogOddsInteger(const float probability)
    {
      const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                           254.f / (kMaxLogOdds - kMinLogOdds)) +
                        1;
      // CHECK_LE(1, value);
      // CHECK_GE(255, value);
      return value;
    }

    class Submap2D /*: public Submap*/
    {
    public:
      //Submap2D(){};

      Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid, ValueConversionTables *conversion_tables);
    
      const Grid2D *grid() const { return grid_.get(); }

      // Insert 'range_data' into this submap using 'range_data_inserter'. The
      // submap must not be finished yet.
      void InsertRangeData(const hjSlam_2d::RangeData &range_data,
                           const ProbabilityGridRangeDataInserter2D *range_data_inserter);
      void Finish();

      // Pose of this submap in the local map frame.
      transform::Rigid3d local_pose() const { return local_pose_; }

      // Number of RangeData inserted.
      int num_range_data() const { return num_range_data_; }
      void set_num_range_data(const int num_range_data)
      {
        num_range_data_ = num_range_data;
      }

      bool insertion_finished() const { return insertion_finished_; }
      void set_insertion_finished(bool insertion_finished)
      {
        insertion_finished_ = insertion_finished;
      }

    private:
      std::unique_ptr<Grid2D> grid_;
      ValueConversionTables *conversion_tables_;

      const transform::Rigid3d local_pose_;
      int num_range_data_ = 0;
      bool insertion_finished_ = false;
    };

    // The first active submap will be created on the insertion of the first range
    // data. Except during this initialization when no or only one single submap
    // exists, there are always two submaps into which range data is inserted: an
    // old submap that is used for matching, and a new one, which will be used for
    // matching next, that is being initialized.
    //
    // Once a certain number of range data have been inserted, the new submap is
    // considered initialized: the old submap is no longer changed, the "new" submap
    // is now the "old" submap and is used for scan-to-map matching. Moreover, a
    // "new" submap gets created. The "old" submap is forgotten by this object.
    class ActiveSubmaps2D
    {
    public:
      ActiveSubmaps2D(/*const proto::SubmapsOptions2D& options*/);
      ~ActiveSubmaps2D(){};

      ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
      ActiveSubmaps2D &operator=(const ActiveSubmaps2D &) = delete;

      // Inserts 'range_data' into the Submap collection.
      std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
          const hjSlam_2d::RangeData &range_data);

      std::vector<std::shared_ptr<const Submap2D>> submaps() const;

    private:
      std::vector<std::shared_ptr<Submap2D>> submaps_;
      std::unique_ptr<ProbabilityGridRangeDataInserter2D> CreateRangeDataInserter();
      std::unique_ptr<Grid2D> CreateGrid(const Eigen::Vector2f &origin);
      void FinishSubmap();
      void AddSubmap(const Eigen::Vector2f &origin);

      // const proto::SubmapsOptions2D options_;
      std::unique_ptr<ProbabilityGridRangeDataInserter2D> range_data_inserter_;
      ValueConversionTables conversion_tables_;
    };

  } // namespace mapping
} // namespace hjSlam_2d

#endif // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
