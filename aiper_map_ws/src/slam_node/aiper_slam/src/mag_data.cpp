#include "mag_data.h"
#include <map>

bool MagData::findNearestMag(const double ts, MagMeasurements &mags, MagData &nearest_mag) {
  double time_interval = 0.01;//10ms
  std::map<double, MagData> mag_near_map;
  mag_near_map.clear();
  for (int j = 0; j < static_cast<int>(mags.size()); ++j) {
    double mag_time = static_cast<double>(mags[j].time) * 1e-6;
    double time_diff = fabs(mag_time - ts);
    if (time_diff < time_interval) {
      mag_near_map.insert(std::make_pair(time_diff, mags[j]));
    }
  }
  if (!mag_near_map.empty()){
    auto it = mag_near_map.begin();
    nearest_mag = it->second;
    return true;
  }
  return false;

}

float MagData::flat_algorithm(float mag[2], float offset[2]) {
//  short temp;
  float Hx, Hy;
//  short old_mag[2] = {0};

  float Vxoff = offset[0];
  float Vyoff = offset[1];
//  short cali_flag = 1;

  float result_yaw = 0.0f;

  offset[0] = Vxoff;
  offset[1] = Vyoff;

  Hx = mag[0] - Vxoff;
  Hy = mag[1] - Vyoff;

  result_yaw = atan2(Hy, Hx) * 180.0f / 3.1416f;
//  if( (Hx == 0) && (Hy < 0) )
//    result_yaw = 90.0f;
//  else if( (Hx == 0) && (Hy > 0) )
//    result_yaw = 270.0f;
//  else if( (Hx < 0) )
//    result_yaw = 180.0f - (atan(Hy / Hx)) * 180.0f / 3.1415f;
//  else if( (Hx > 0) && (Hy < 0) )
//    result_yaw = -(atan(Hy / Hx)) * 180.0f / 3.1415f;
//  else if( (Hx > 0) && (Hy >= 0) )
//    result_yaw = 360.0f - (atan(Hy / Hx)) * 180.0f / 3.1415f;
  return result_yaw;

}

float MagData::calculateMagDegree(const float cur_mag_x, const float cur_mag_y,
        const float offset_x, const float offset_y) {

  float mag_cur[2] = {cur_mag_x, cur_mag_y};
  float offset[2] = {offset_x, offset_y};
  float result_yaw = flat_algorithm(mag_cur, offset);
  return result_yaw;

}