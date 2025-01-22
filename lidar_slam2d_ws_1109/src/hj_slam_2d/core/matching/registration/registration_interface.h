/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

// #include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
// #include "cloud_data.hpp"
#include "../../../util/point_cloud.h"
// #include <memory>
// #include <unique_ptr.h>

namespace hjSlam_2d
{
  class RegistrationInterface
  {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool setInputTarget(const hjSlam_2d::PointCloud &input_target) = 0;
    virtual bool scanMatch(const hjSlam_2d::PointCloud &input_source) = 0;
  };
}

#endif