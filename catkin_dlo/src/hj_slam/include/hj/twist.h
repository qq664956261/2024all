
// created by: zhangcheng
#ifndef TWIST_HH
#define TWIST_HH

#include "Eigen/Eigen"
#include "header.h"

class Twist {
public:
    Twist() {
        linear.setIdentity();
        angular.setIdentity();
    };

    Twist(Eigen::Vector3d &linear_, Eigen::Vector3d &angular_) : linear(linear_), angular(angular_) {};
    
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
};

#endif //TWIST_HH
