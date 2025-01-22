/*
 * @Description: UWB数据
 * @Author: zhang cheng
 * @Date: 2024-04-11 09:50:30
 */

#ifndef MAPPING_UWB_H
#define MAPPING_UWB_H

#include <memory>
#include "localization/common/eigen_types.h"

namespace sad
{
    /// UWB 读数
    struct UWB
    {
        UWB() = default;
        UWB(double t, const int &idA, const int &idB, const double &data) : timestamp_(t), idA_(idA), idB_(idB) , data_(data){}

        double timestamp_ = 0.0;
        int idA_ = 0;
        int idB_ = 0;
        double data_ = 0.0;
    };
} // namespace sad

using UWBPtr = std::shared_ptr<sad::UWB>;

#endif // MAPPING_UWB_H
