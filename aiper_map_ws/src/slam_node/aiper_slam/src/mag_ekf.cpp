#include "mag_ekf.h"
#include <iostream>

Yaw_Kalman_Filter::Yaw_Kalman_Filter(){
  //参数初始化设置
  //系统状态矩阵
  A<< 1;
  //协方差
  P << 1;

  //测量矩阵
  H<< 1;

  //（预测）过程噪音
  Q<< 0.03;

  //真实传感器噪音
  R<<5;

}

Yaw_Kalman_Filter::~Yaw_Kalman_Filter(){}


TypeVector1 Yaw_Kalman_Filter::predictAndupdate(TypeVector1 pre, TypeVector1 z){


  X = A * pre; // 状态一步预测方程
  P = A * P * (A.transpose()) + Q; //一步预测协方差阵
  TypeVector1 K = P * (H.transpose()) * ((H * P * (H.transpose()) + R).inverse()); //kalman增益
  X = X + K * (z - H * X); //状态更新：
  int x_size = X.size();
  TypeVector1 I = Eigen::MatrixXd::Identity(x_size, x_size);
  P = (I - K * H) * P; // 协方差阵更新：
  return X;
}


TypeVector1 Yaw_Kalman_Filter::getX(){
  return X;
}


