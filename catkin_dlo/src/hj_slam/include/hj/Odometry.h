// created by: zhangcheng
#ifndef ODOMETRY_HH
#define ODOMETRY_HH
#include "pose.h"
#include "twist.h"

class Odometry{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Odometry()=default;
    Odometry(Pose &pose_,Twist &twist_):pose(pose_),twist(twist_){};
    Header header;
    Pose pose;
    Twist twist;
    

    int seq;
};
#endif //ODOMETRY_HH
