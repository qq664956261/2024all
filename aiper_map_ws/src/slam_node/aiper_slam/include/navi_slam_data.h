#pragma once

#include <deque>
#include <math.h>
#include <iostream>

struct NaviSlamData;
typedef std::deque<NaviSlamData> NaviSlamDataMeasurements;
struct NaviSlamData {
  uint64_t time_;
  uint8_t mode_;
  uint8_t rotation_state_;

  NaviSlamData() {}
  NaviSlamData(const uint64_t timestamp, uint8_t mode, uint8_t rotation_state)
        : time_(timestamp), mode_(mode), rotation_state_(rotation_state) {}

};
