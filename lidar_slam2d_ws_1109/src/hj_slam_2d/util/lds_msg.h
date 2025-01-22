#ifndef MY_IDS_MSG_H
#define MY_IDS_MSG_H

#include <vector>
namespace hjSlam_2d
{
    struct Pose
    {
        float x;
        float y;
        float theta;
    };

    struct LdsPoint
    {
        float x;
        float y;
        float rho;
        float theta;
        float power;
    };

    struct PredictPose
    {
        Pose predictPose;
        Pose pose;
    };

    struct Lds
    {
        std::vector<LdsPoint> points;
        PredictPose ldsPose;
    };
}

#endif  // MY_IDS_MSG_H