#pragma once

#include "utils.h"
#include <memory>
#include <list>

class ArcFusion;
typedef std::shared_ptr<ArcFusion> ArcFusionPtr;
class MapPoint;
typedef std::shared_ptr<MapPoint> MapPointPtr;
typedef std::list<MapPointPtr> MapPointList;

class ArcFusion {
 public:
  ArcFusion();
  ~ArcFusion();

  float radius;  // 圆弧的半径
  float yaw;    // 此时机器人的里程计yaw角
  MapPointPtr circle_center_odom; // 此时机器人的里程计圆心
  MapPointPtr circle_center_sonar;// 此时机器人的超声波圆心
//  MapPointPtr first_data; // 超声波弧数据的起点
//  MapPointPtr core_data; // 超声波弧数据的中点，核心数据
//  MapPointPtr back_data;  // 超声波弧数据的终点
  MapPointList circle_data;// 离散化的超声波数据

 private:
};