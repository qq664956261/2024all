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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {
// 用于在 2D 地图上执行实时的相关性扫描匹配（Real-time Correlative Scan Matching）。该算法用于根据激光雷达的点云数据和地图（如概率网格或 TSDF）来优化机器人的位姿估计。
// 计算点云在指定像素坐标位置下与TSDF2D地图匹配的得分
float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =
        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}

// 计算点云在指定像素坐标位置下与ProbabilityGrid地图匹配的得分
float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    // 对每个点都加上像素坐标的offset, 相当于对点云进行平移
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    // 获取占用的概率
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    // 以概率为得分
    candidate_score += probability;
  }
  // 计算平均得分
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 生成所有的候选解
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  // 计算候选解的个数
  // num_scans旋转后的点云集合的个数
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_��8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����8�¢q�{ʞ���]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T����]�lC���2�"T���测出的平移量进行平移, 获取平移后的点在地图中的索引
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  // Step: 4 生成所有的候选解
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  
  // Step: 5 计算所有候选解的加权得分
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  // Step: 6 获取最优解
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  
  // Step: 7 将计算出的偏差量加上原始位姿获得校正后的位姿
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

// 计算所有候选解的加权得分
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  // 通过 for 循环遍历每一个候选解（Candidate2D）。
  // 每个 Candidate2D 对象包含了一个扫描的偏移量（x_index_offset, y_index_offset）以及与该扫描相关的一些参数。
  for (Candidate2D& candidate : *candidates) {
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        candidate.score = ComputeCandidateScore(
            static_cast<const ProbabilityGrid&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
      case GridType::TSDF:
        candidate.score = ComputeCandidateScore(
            static_cast<const TSDF2D&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
    }
    // 对得分进行加权
    // 以鼓励较小的偏移量（包括平移和旋转）优先于较大的偏移量。
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
