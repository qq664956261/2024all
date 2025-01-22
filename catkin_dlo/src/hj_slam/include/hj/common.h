
// created by: zhangcheng
#ifndef COMMON_H
#define COMMON_H
#include <cmath>

#define M_PI 3.1415926

template<typename T>
T NormalizeAngle(T angle){
    if (angle >  M_PI){
        angle -= M_PI * 2;
    }else if(angle <= -M_PI){
        angle += M_PI * 2;
    }
    return angle;
}
template<typename T>
T SetMin(T x, T min){
    x = x < min? min : x;
    return x;
}
template<typename T>
T SetMax(T x, T max){
    x = x > max? max : x;
    return x;
}
template<typename T>
double GetRosTime(T msg){
    return msg.header.stamp.toSec();
}
#endif //COMMON_H