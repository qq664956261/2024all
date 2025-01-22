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
// ç”¨äºåœ¨ 2D åœ°å›¾ä¸Šæ‰§è¡Œå®æ—¶çš„ç›¸å…³æ€§æ‰«æåŒ¹é…ï¼ˆReal-time Correlative Scan Matchingï¼‰ã€‚è¯¥ç®—æ³•ç”¨äºæ ¹æ®æ¿€å…‰é›·è¾¾çš„ç‚¹äº‘æ•°æ®å’Œåœ°å›¾ï¼ˆå¦‚æ¦‚ç‡ç½‘æ ¼æˆ– TSDFï¼‰æ¥ä¼˜åŒ–æœºå™¨äººçš„ä½å§¿ä¼°è®¡ã€‚
// è®¡ç®—ç‚¹äº‘åœ¨æŒ‡å®šåƒç´ åæ ‡ä½ç½®ä¸‹ä¸TSDF2Dåœ°å›¾åŒ¹é…çš„å¾—åˆ†
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

// è®¡ç®—ç‚¹äº‘åœ¨æŒ‡å®šåƒç´ åæ ‡ä½ç½®ä¸‹ä¸ProbabilityGridåœ°å›¾åŒ¹é…çš„å¾—åˆ†
float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    // å¯¹æ¯ä¸ªç‚¹éƒ½åŠ ä¸Šåƒç´ åæ ‡çš„offset, ç›¸å½“äºå¯¹ç‚¹äº‘è¿›è¡Œå¹³ç§»
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    // è·å–å ç”¨çš„æ¦‚ç‡
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    // ä»¥æ¦‚ç‡ä¸ºå¾—åˆ†
    candidate_score += probability;
  }
  // è®¡ç®—å¹³å‡å¾—åˆ†
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// ç”Ÿæˆæ‰€æœ‰çš„å€™é€‰è§£
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  // è®¡ç®—å€™é€‰è§£çš„ä¸ªæ•°
  // num_scansæ—‹è½¬åçš„ç‚¹äº‘é›†åˆçš„ä¸ªæ•°
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚ºØ8ıÂ¢qƒ{Ê½Îî]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚î]œlC¾‡ì2Ñ"TØô‚æµ‹å‡ºçš„å¹³ç§»é‡è¿›è¡Œå¹³ç§», è·å–å¹³ç§»åçš„ç‚¹åœ¨åœ°å›¾ä¸­çš„ç´¢å¼•
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  // Step: 4 ç”Ÿæˆæ‰€æœ‰çš„å€™é€‰è§£
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  
  // Step: 5 è®¡ç®—æ‰€æœ‰å€™é€‰è§£çš„åŠ æƒå¾—åˆ†
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  // Step: 6 è·å–æœ€ä¼˜è§£
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  
  // Step: 7 å°†è®¡ç®—å‡ºçš„åå·®é‡åŠ ä¸ŠåŸå§‹ä½å§¿è·å¾—æ ¡æ­£åçš„ä½å§¿
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

// è®¡ç®—æ‰€æœ‰å€™é€‰è§£çš„åŠ æƒå¾—åˆ†
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  // é€šè¿‡ for å¾ªç¯éå†æ¯ä¸€ä¸ªå€™é€‰è§£ï¼ˆCandidate2Dï¼‰ã€‚
  // æ¯ä¸ª Candidate2D å¯¹è±¡åŒ…å«äº†ä¸€ä¸ªæ‰«æçš„åç§»é‡ï¼ˆx_index_offset, y_index_offsetï¼‰ä»¥åŠä¸è¯¥æ‰«æç›¸å…³çš„ä¸€äº›å‚æ•°ã€‚
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
    // å¯¹å¾—åˆ†è¿›è¡ŒåŠ æƒ
    // ä»¥é¼“åŠ±è¾ƒå°çš„åç§»é‡ï¼ˆåŒ…æ‹¬å¹³ç§»å’Œæ—‹è½¬ï¼‰ä¼˜å…ˆäºè¾ƒå¤§çš„åç§»é‡ã€‚
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
