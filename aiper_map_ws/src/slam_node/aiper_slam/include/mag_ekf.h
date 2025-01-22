#pragma once
#include <eigen3/Eigen/Dense>

// #define PI 3.1416
typedef typename Eigen::Matrix<double, 1, 1> TypeVector1;
class Yaw_Kalman_Filter{
  public:
    Yaw_Kalman_Filter();
    TypeVector1 predictAndupdate(TypeVector1 pre, TypeVector1 z);
    TypeVector1 getX();
    ~Yaw_Kalman_Filter();
private:
    TypeVector1 A; //系统状态矩阵
    TypeVector1 P; //协方差
    TypeVector1 Q; //测量过程噪音（预测）
    TypeVector1 R; //真实传感器噪音
    TypeVector1 H; //测量矩阵
    TypeVector1 X =  TypeVector1::Zero();

    bool isinitized = false; //判断是否进行了初始化

};
