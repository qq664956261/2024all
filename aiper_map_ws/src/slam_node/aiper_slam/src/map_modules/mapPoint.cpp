#include "map_modules/mapPoint.h"

uint64_t MapPoint::counter = 0;

MapPoint::MapPoint() {

};

MapPoint::MapPoint(const float x, const float y, const float z)
  : id(counter++), cor_x(x), cor_y(y), cor_z(z){

}

float MapPoint::getCor_x() {
  return cor_x;
}

void MapPoint::alterCor_x(float alter_x) {
  cor_x = alter_x;
}

float MapPoint::getCor_y() {
  return cor_y;
}

void MapPoint::alterCor_y(float alter_y) {
  cor_y = alter_y;
}

float MapPoint::getCor_z() {
  return cor_z;
}

void MapPoint::alterCor_z(float alter_z) {
  cor_z = alter_z;
}

uint64_t MapPoint::getID() {
  return id;
}

MapPoint::~MapPoint() {

};