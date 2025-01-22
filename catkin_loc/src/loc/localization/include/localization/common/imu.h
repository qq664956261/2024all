/*
 * @Description: IMU数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:30
 */

#ifndef MAPPING_IMU_H
#define MAPPING_IMU_H

#include <memory>
#include "localization/common/eigen_types.h"

namespace sad
{

    struct Orientation
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    };
    /// IMU 读数
    struct IMU
    {
        IMU() = default;
        IMU(double t, const Vec3d &gyro, const Vec3d &acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

        double timestamp_ = 0.0;
        Vec3d gyro_ = Vec3d::Zero();
        Vec3d acce_ = Vec3d::Zero();
        Orientation orientation;
    };
} // namespace sad

using IMUPtr = std::shared_ptr<sad::IMU>;

#endif // MAPPING_IMU_H
