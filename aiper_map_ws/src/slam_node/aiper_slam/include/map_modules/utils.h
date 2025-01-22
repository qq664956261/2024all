#ifndef UTILS_H
#define UTILS_H
#include "mapPoint.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <math.h>

enum SonarStatus {
    front, back
};


struct ReceivedSonar {
  uint64_t timestamp = 0; //us
  float distance = 0.;
  SonarStatus status;
};

struct ReceivedTof {
  uint64_t timestamp = 0; //us
  float front_distance = 0.;
  float back_distance = 0.;
};

struct Odom {
  uint64_t timestmap;
  float x;
  float y;
  float z;
  float yaw;
  float pitch;
  float roll;
  Odom() {}
  Odom(const uint64_t timestmap_, const float x_, const float y_, const float z_,
    const float yaw_, const float pitch_, const float roll_) :
    timestmap(timestmap_), x(x_), y(y_), z(z_),
    yaw(yaw_), pitch(pitch_), roll(roll_) {

  }
    Odom& operator = (const Odom& other) {
      if(this != &other) {
        this->timestmap = other.timestmap;
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->yaw = other.yaw;
        this->pitch = other.pitch;
        this->roll = other.roll;
      }
      return *this;
    }
    bool operator != (const Odom& other) {
      return !(this->x == other.x && this->y == other.y && this->z == other.z &&
               this->yaw == other.yaw && this->pitch == other.pitch && this->roll == other.roll);
    }
    bool operator == (const Odom& other) {
      return this->x == other.x && this->y == other.y && this->z == other.z &&
               this->yaw == other.yaw && this->pitch == other.pitch && this->roll == other.roll;
    }
    // 计算两点之间完整距离的方法
    float calDistance(const Odom& other) const {
      return std::sqrt(std::pow(this->x - other.x, 2) +
                       std::pow(this->y - other.y, 2) +
                       std::pow(this->z - other.z, 2));
    }

    template<typename T>
    T toEigenIsometry() const {
      Eigen::Vector3f v(this->x, this->y, this->z);
      Eigen::Quaternionf r = Eigen::AngleAxisf(this->yaw, Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(this->pitch, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(this->roll, Eigen::Vector3f::UnitX());
      Eigen::Matrix3f rotation_matrix = r.toRotationMatrix();
      T eigen_isometry = T::Identity();
      eigen_isometry.rotate(rotation_matrix);
      eigen_isometry.pretranslate(v);
      return eigen_isometry;
    }

};

struct Sonar {
  float sonar_base_x;
  float sonar_base_y;
  float sonar_base_yaw;
};

struct Point2D {
  float x;
  float y;
  Point2D() {}
  Point2D(const float x_, const float y_) : x(x_), y(y_) {
  }
  float dot(Point2D p) {
    return x * p.x + y * p.y;
  }

};

struct Point3D {
  float x;
  float y;
  float z;

  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {
  }

  float dot(Point3D p) {
    return x * p.x + y * p.y;
  }
};

struct SynchronizedData {
  Odom odom;
  MapPoint map_point;
//  uint64_t odom_time;
};
typedef std::shared_ptr<SynchronizedData> SynchronizedDataPtr;
typedef std::vector<SynchronizedDataPtr> SynchronizedDataPtrVec;

#endif
