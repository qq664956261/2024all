#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "../grid/smmap.h"
#include "../utils/macro_params.h"
// #include "hj_slam/Lds.h"
#include "../../../../util/lds_msg.h"

#define LASER_MAXBEAMS 2048

namespace HJ_2dLaserMapping
{
    class ScanMatcher
    {
    public:
        ScanMatcher();
        ~ScanMatcher();

        //void registerScan(ScanMatcherMap &map, const hj_slam::Lds &msg);

        void registerScan2(ScanMatcherMap &map, const hjSlam_2d::Lds &msg);

    private:
        IntPoint *m_linePoints;

        // 定义一大堆参数以及其set和get函数
        PARAM_SET_GET(double, laserMaxRange, protected, public, public) // 激光的最大测距范围
        PARAM_SET_GET(double, usableRange, protected, public, public)   // 使用的激光的最大范围
        PARAM_SET_GET(double, gaussianSigma, protected, public, public)
        PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
        PARAM_SET_GET(int, kernelSize, protected, public, public)
        PARAM_SET_GET(double, optAngularDelta, protected, public, public)              // 优化时的角度增量
        PARAM_SET_GET(double, optLinearDelta, protected, public, public)               // 优化时的长度增量
        PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public) // 优化时的迭代次数
        PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
        PARAM_SET_GET(bool, generateMap, protected, public, public)
        PARAM_SET_GET(double, enlargeStep, protected, public, public)
        PARAM_SET_GET(double, fullnessThreshold, protected, public, public)          // 被认为是占用的阈值
        PARAM_SET_GET(double, angularOdometryReliability, protected, public, public) // 里程计的角度可靠性
        PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)  // 里程计的长度可靠性
        PARAM_SET_GET(double, freeCellRatio, protected, public, public)              // free和occupany的阈值
        PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)     // 去掉初始的几个激光束的数量
    };

};

#endif
