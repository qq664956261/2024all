#pragma once

#include <deque>
#include <math.h>

struct LeftBackUltraData;
typedef std::deque<UltraData> LeftBackUltraDataMeasurements;
struct LeftBackUltraData {
  uint64_t time_;
  uint32_t distance_;
  uint8_t status_;
  LeftBackUltraData() {}
  LeftBackUltraData(const uint64_t timestamp, uint32_t distance, uint8_t status)
        : time_(timestamp), distance_(distance), status_(status){}

};


