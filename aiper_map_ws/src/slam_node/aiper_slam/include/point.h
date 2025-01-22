#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>

class Point;
typedef std::shared_ptr<Point> PointPtr;

class Point {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //static uint64_t counter_;
  uint64_t id_;                    //!< Unique ID of the point.
  Eigen::Vector3d pos_;            //!< 3d pos of the point in the world coordinate frame.
//
  Point(const Eigen::Vector3d& pos, const uint64_t id)
          : id_(id),
            pos_(pos){}

  ~Point() {}

};


