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

#include "probability_values.h"
#include <memory>

namespace hjSlam_2d {
namespace mapping {

namespace {
// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  //CHECK_GE(value, 0);
  //CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = std::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds)
{
std::vector<uint16> result;
result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                 kUpdateMarker);

for (int cell = 1; cell != 32768; ++cell)
{
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                       odds * Odds((*kValueToProbability)[cell]))) +
                   kUpdateMarker);
}

return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {

  std::vector<uint16> result;
  //free 映射到整数值 
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);

  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace hjSlam_2d
