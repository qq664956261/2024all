#include "pressure_data.h"

float PressureData::calDeltaZ(const PressureData &last_pressure, const PressureData &curr_pressure) {
  int last_pascal = static_cast<int>(last_pressure.pressure);
  int curr_pascal = static_cast<int>(curr_pressure.pressure);
  int delta_pascal = curr_pascal - last_pascal;
  float delta_z = -delta_pascal / 10000.0;
  return delta_z;

}

