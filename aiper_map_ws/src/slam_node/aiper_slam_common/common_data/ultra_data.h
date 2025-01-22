#pragma once

#include <deque>
#include <cstdint>

namespace common_data {
struct UltraData;
typedef std::deque<UltraData> UltraDataMeasurements;
struct UltraData {
  uint64_t time_;  ///< In seconds.
  uint32_t front_l_;
  uint32_t front_m_;
  uint32_t front_r_;
  uint8_t status_;
  UltraData() {}
  UltraData(const uint64_t timestamp, uint32_t front_l, uint32_t front_m, uint32_t front_r, uint8_t status)
        : time_(timestamp), front_l_(front_l), front_m_(front_m), front_r_(front_r), status_(status){}

};

}  // namespace common_data