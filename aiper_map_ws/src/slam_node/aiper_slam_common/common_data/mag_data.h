#pragma once

#include <deque>
#include <cmath>
#include <iostream>

namespace common_data {
struct MagData;
typedef std::deque<MagData> MagMeasurements;
struct MagData {
  MagData() {}
  MagData(const uint64_t timestamp, float mag_x_, float mag_y_, float mag_z_, float yaw_)
          : time(timestamp), mag_x(mag_x_), mag_y(mag_y_), mag_z(mag_z_), yaw{yaw_} {}
  bool findNearestMag(const double ts, MagMeasurements &mags, MagData &nearest);
  void flat_algorithm(float mag[2], float offset[2]);
  void calculateMagDegree(const float cur_mag_x, const float cur_mag_y,
                          const float offset_x, const float offset_y);
  uint64_t time;  ///< In seconds.
  float mag_x;
  float mag_y;
  float mag_z;
  float yaw;
};

}  // namespace common_data