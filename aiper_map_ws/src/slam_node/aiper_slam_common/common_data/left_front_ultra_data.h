#pragma once

#include <deque>
#include <math.h>

namespace common_data {
struct LeftFrontUltraData;
typedef std::deque<UltraData> LeftFrontUltraDataMeasurements;
struct LeftFrontUltraData {
  uint64_t time_;
  uint32_t distance_;
  uint8_t status_;
  LeftFrontUltraData() {}
  LeftFrontUltraData(const uint64_t timestamp, uint32_t distance, uint8_t status)
        : time_(timestamp), distance_(distance), status_(status){}

};
}  // namespace common_data