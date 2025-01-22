#include "mag_data.h"
#include <map>
namespace common_data {
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

void MagData::flat_algorithm(float mag[2], float offset[2])
{
//  short temp;
  float Hx, Hy;
//  short old_mag[2] = {0};

  float Vxoff = offset[0];
  float Vyoff = offset[1];
//  short cali_flag = 1;

  yaw = 0.0f;

  offset[0] = Vxoff;
  offset[1] = Vyoff;

  Hx = mag[0] - Vxoff;
  Hy = mag[1] - Vyoff;

  if( (Hx == 0) && (Hy < 0) )
    yaw = 90.0f;
  else if( (Hx == 0) && (Hy > 0) )
    yaw = 270.0f;
  else if( (Hx < 0) )
    yaw = 180.0f - (atan(Hy / Hx)) * 180.0f / 3.1415f;
  else if( (Hx > 0) && (Hy < 0) )
    yaw = -(atan(Hy / Hx)) * 180.0f / 3.1415f;
  else if( (Hx > 0) && (Hy > 0) )
    yaw = 360.0f - (atan(Hy / Hx)) * 180.0f / 3.1415f;

//  old_mag[0] = mag[0];
//  old_mag[1] = mag[1];
}

void MagData::calculateMagDegree(const float cur_mag_x, const float cur_mag_y,
        const float offset_x, const float offset_y){

  float mag_cur_[2] = {cur_mag_x, cur_mag_y};
  float offset_[2] = {offset_x, offset_y};
  flat_algorithm(mag_cur_, offset_);

}
}