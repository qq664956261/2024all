
// 该文件定义了系统中用到的点和点云类型，以及点云到Eigen的常用的转换函数

#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include "ExtrinsicErrorTerm/pointType.h"
#include "ExtrinsicErrorTerm/common/eigen_types.h"

namespace sad {

// 定义系统中用到的点和点云类型
using PointType = mypcl::PointXYZI;
using PointCloudType = mypcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType& pt) 
{
    Eigen::Vector3f p;
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z; 
    return p; 
}
inline Vec3d ToVec3d(const PointType& pt) 
{ 
    Eigen::Vector3d p;
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;
    return p; 
}

// 模板类型转换函数
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    return p;
}



}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_POINT_TYPES_H
