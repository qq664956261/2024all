#pragma once

#include <deque>
#include <math.h>

namespace common_data {
struct LeftTofData;
typedef std::deque<LeftTofData> LeftTofDataMeasurements;
struct LeftTofData {
  uint64_t time_;
  uint32_t front_distance_;
  uint32_t back_distance_;
  LeftTofData() {}
  LeftTofData(const uint64_t timestamp, uint32_t front_distance, uint32_t back_distance)
        : time_(timestamp), front_distance_(front_distance), back_distance_(back_distance){}

};
}  // namespace common_data
