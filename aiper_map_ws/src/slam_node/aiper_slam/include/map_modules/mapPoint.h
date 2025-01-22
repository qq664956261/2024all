#pragma once

#include <iostream>
#include <memory>
#include <list>
#include <unordered_map>
#include <tuple>
#include <math.h>

class MapPoint;
typedef std::shared_ptr<MapPoint> MapPointPtr;

class MapPoint {
 public:
  MapPoint();
  MapPoint(const float x, const float y, const float z);
  static uint64_t counter;

  ~MapPoint();

  float getCor_x();
  float getCor_y();
  float getCor_z();
  void alterCor_x(float alter_x);
  void alterCor_y(float alter_y);
  void alterCor_z(float alter_z);
    // 将点映射到格子的函数
  std::tuple<int, int, int> pointToGrid(const MapPointPtr& point, double gridSize) {
    return std::make_tuple(
            static_cast<int>((point->cor_x / gridSize) + 0.5),
            static_cast<int>((point->cor_y / gridSize) + 0.5),
            static_cast<int>((point->cor_z / gridSize) + 0.5)
    );
  }

  uint64_t hash(const MapPointPtr& point, double gridSize) {
    int hash_x = static_cast<int>((point->cor_x / gridSize) + 0.5);
    int hash_y = static_cast<int>((point->cor_y / gridSize) + 0.5);
    int hash_z = static_cast<int>((point->cor_z / gridSize) + 0.5);

    const uint64_t prime1 = 73856093;
    const uint64_t prime2 = 19349663;
    const uint64_t prime3 = 83492791;

    uint64_t h1 = std::hash<int>{}(hash_x) * prime1;
    uint64_t h2 = std::hash<int>{}(hash_y) * prime2;
    uint64_t h3 = std::hash<int>{}(hash_z) * prime3;

    return h1 ^ h2 ^ h3;
  }

  uint64_t getID();

    // 计算两点之间完整距离的方法
  float distance(const MapPoint& other) const {
    return std::sqrt(std::pow(cor_x - other.cor_x, 2) +
                     std::pow(cor_y - other.cor_y, 2) +
                     std::pow(cor_z - other.cor_z, 2));
  }


private:
  uint64_t id;
  float cor_x;
  float cor_y;
  float cor_z;
};