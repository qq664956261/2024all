#pragma once

#include <deque>
#include <math.h>
#include <iostream>

struct PressureData;
typedef std::deque<PressureData> PressureMeasurements;
struct PressureData {
  uint64_t time;
  int32_t pressure;
  int32_t temperature;

  PressureData() { }
  PressureData(const uint64_t timestamp, int32_t pressure_, int32_t temperature_)
               : time(timestamp), pressure(pressure_), temperature(temperature_) { }
  float calDeltaZ(const PressureData &last_pressure, const PressureData &curr_pressure);

};

