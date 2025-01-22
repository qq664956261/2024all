// created by: zhangcheng
#ifndef POINTUNDISTORT_HH
#define POINTUNDISTORT_HH

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>
#include <boost/circular_buffer.hpp>

struct IMUData
{
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
class PointUndistort
{
public:
    PointUndistort();
    ~PointUndistort();
    void setIMUBuffer(boost::circular_buffer<IMUData> imu_buffer)
    {
        imu_buffer_.clear();
        imu_buffer_ = imu_buffer;
    }
    void setLidarTime(double start_time, double end_time, double time_increment)
    {
        lidar_start_time_ = start_time;
        lidar_end_time_ = end_time;
        time_increment_ = time_increment;
    }
    Eigen::Vector3d LinearInterpolation(const float &t, const Eigen::Vector3d &t1, const Eigen::Vector3d &t2);
    Eigen::Vector3d ImuLinearInterpolation(const double &t, const IMUData &t1, const IMUData &t2);
    pcl::PointXYZI PointTransform(const Eigen::Matrix4d &pose, const pcl::PointXYZI &point);
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetUndistortedPointCloudByIMU(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar);

private:
    boost::circular_buffer<IMUData> imu_buffer_;
    double lidar_start_time_;
    double lidar_end_time_;
    double time_increment_;
};

#endif // POINTUNDISTORT_HH