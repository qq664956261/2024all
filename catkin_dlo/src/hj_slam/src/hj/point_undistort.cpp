// created by: zhangcheng
#include "hj/point_undistort.h"

PointUndistort::PointUndistort()
{
    imu_buffer_.set_capacity(1000);
}

PointUndistort::~PointUndistort()
{
}

Eigen::Vector3d PointUndistort::LinearInterpolation(const float &t, const Eigen::Vector3d &t1, const Eigen::Vector3d &t2)
{
    Eigen::Vector3d deltaDis = t2 - t1;
    Eigen::Vector3d interpolatedVal = t1 + t * deltaDis;
    return interpolatedVal;
}

Eigen::Vector3d PointUndistort::ImuLinearInterpolation(const double &t, const IMUData &t1, const IMUData &t2)
{
    Eigen::Vector3d w1 = Eigen::Vector3d(t1.gyro.x(), t1.gyro.y(), t1.gyro.z());
    Eigen::Vector3d w2 = Eigen::Vector3d(t2.gyro.x(), t2.gyro.y(), t2.gyro.z());
    return LinearInterpolation(t, w1, w2);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointUndistort::GetUndistortedPointCloudByIMU(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar)
{

    double lidar_size = lidar->points.size();
    // std::cout<<"lidar_size: "<<lidar_size<<std::endl;
    // std::cout<<"imu_buffer_size: "<<imu_buffer_.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr refertoStart(new pcl::PointCloud<pcl::PointXYZI>());
    refertoStart->header.stamp = lidar->header.stamp;
    refertoStart->height = lidar->height;
    refertoStart->width = lidar->width;
    refertoStart->header.seq = lidar->header.seq;
    refertoStart->is_dense = false;
    refertoStart->points.reserve(lidar_size);

    double t = .0;
    Eigen::Quaterniond Todom_ls = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q2 = Eigen::Quaterniond::Identity();

    IMUData startImu;
    pcl::PointXYZI tmpPoint;
    Eigen::Vector3d w_ls = Eigen::Vector3d::Zero();
    int moveLeftImuIndice = 0;
    int moveRightImuIndice = 0;

    {

        const int ImuDequeSize = imu_buffer_.size();
        // std::cout<<"ImuDequeSize: "<<ImuDequeSize<<std::endl;
        // std::cout << "lidar_start_time_: " << std::to_string(lidar_start_time_) << std::endl;
        // std::cout << "imu_buffer_.front().timestamp: " << std::to_string(imu_buffer_.front().timestamp) << std::endl;
        // std::cout << "imu_buffer_.back().timestamp: " << std::to_string(imu_buffer_.back().timestamp) << std::endl;
        for (int i = 0; i < ImuDequeSize; ++i)
        {
            if ((lidar_start_time_ >= imu_buffer_[i].timestamp) &&
                (imu_buffer_[i + 1].timestamp >= lidar_start_time_))
            {
                moveLeftImuIndice = i;
                moveRightImuIndice = i + 1;
                t = (double)(lidar_start_time_ - imu_buffer_[i].timestamp) / (double)(imu_buffer_[i + 1].timestamp - imu_buffer_[i].timestamp);

                Eigen::Vector3d w1 = Eigen::Vector3d(imu_buffer_[i].gyro.x(), imu_buffer_[i].gyro.y(), imu_buffer_[i].gyro.z());
                Eigen::Vector3d w2 = Eigen::Vector3d(imu_buffer_[i + 1].gyro.x(), imu_buffer_[i + 1].gyro.y(), imu_buffer_[i + 1].gyro.z());
                Eigen::Vector3d w = LinearInterpolation(t, w1, w2);
                w_ls = w;

                //   Eigen::Vector3d acc1 = Eigen::Vector3d(imuDeuque[i].lin_accel.x, imuDeuque[i].lin_accel.y, imuDeuque[i].lin_accel.z);
                //   Eigen::Vector3d acc2 = Eigen::Vector3d(imuDeuque[i+1].lin_accel.x, imuDeuque[i+1].lin_accel.y, imuDeuque[i+1].lin_accel.z);
                //   Eigen::Vector3d acc = LinearInterpolation(t, acc1, acc2);

                startImu.timestamp = lidar_start_time_;
                startImu.gyro.x() = w[0];
                startImu.gyro.y() = w[1];
                startImu.gyro.z() = w[2];
                //   startImu.lin_accel.x = acc[0];
                //   startImu.lin_accel.y = acc[1];
                //   startImu.lin_accel.z = acc[2];

                // 把该点放到点云里
                tmpPoint.x = lidar->points[0].x;
                tmpPoint.y = lidar->points[0].y;
                tmpPoint.z = lidar->points[0].z;
                tmpPoint.intensity = lidar->points[0].intensity;
                refertoStart->points.emplace_back(tmpPoint);

                break;
            }
        }

        double currentPointStamp = 0;
        Eigen::Quaterniond qls_lc = Eigen::Quaterniond::Identity();
        Eigen::Vector3d Ang_lc = Eigen::Vector3d::Zero();

        Eigen::Matrix4d Tls_lc;
        Tls_lc.setIdentity();
        Eigen::Matrix4d Tlc_lc;
        Tlc_lc.setIdentity();
        double sum_time = 0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        for (int i = 1; i < lidar_size; ++i)
        {
            currentPointStamp = lidar_start_time_ + double(time_increment_ * i);
            for (int j = moveLeftImuIndice; j < ImuDequeSize - 1; ++j)
            {
                // std::cout << "currentPointStamp: " << std::to_string(currentPointStamp) << std::endl;
                // std::cout << "imu_buffer_[j].timestamp: " << std::to_string(imu_buffer_[j].timestamp) << std::endl;
                // std::cout << "imu_buffer_[j + 1].timestamp: " << std::to_string(imu_buffer_[j + 1].timestamp) << std::endl;
                if ((currentPointStamp >= imu_buffer_[j].timestamp) &&
                    (imu_buffer_[j + 1].timestamp >= currentPointStamp))
                {
                    moveLeftImuIndice = j;
                    moveRightImuIndice = j + 1;

                    t = (double)(currentPointStamp - imu_buffer_[j].timestamp) / (double)(imu_buffer_[j + 1].timestamp - imu_buffer_[j].timestamp);
                    Ang_lc = ImuLinearInterpolation(t, imu_buffer_[j], imu_buffer_[j + 1]);
                    Eigen::Vector3d angle = (Ang_lc + w_ls) / 2;
                    // std::cout << "angle: " << angle << std::endl;
                    Eigen::Vector3d deltaAngle = Eigen::Vector3d::Zero();

                    Eigen::Quaterniond qq = q;
                    q.w() -= 0.5 * (qq.x() * angle.x() + qq.y() * angle.y() + qq.z() * angle.z()) * time_increment_;
                    q.x() += 0.5 * (qq.w() * angle.x() - qq.z() * angle.y() + qq.y() * angle.z()) * time_increment_;
                    q.y() += 0.5 * (qq.z() * angle.x() + qq.w() * angle.y() - qq.x() * angle.z()) * time_increment_;
                    q.z() += 0.5 * (qq.x() * angle.y() - qq.y() * angle.x() + qq.w() * angle.z()) * time_increment_;
                    sum_time = sum_time + time_increment_;
                    double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
                    q.w() /= norm;
                    q.x() /= norm;
                    q.y() /= norm;
                    q.z() /= norm;
                    // Tlc_lc.block(0, 0, 3, 3) = q.toRotationMatrix();
                    // std::cout<<"q.toRotationMatrix():"<<q.toRotationMatrix()<<std::endl;

                    w_ls = Ang_lc;

                    // Tls_lc = Tls_lc * Tlc_lc;
                    Tls_lc.block(0, 0, 3, 3) = q.toRotationMatrix();

                    tmpPoint = PointTransform(Tls_lc, lidar->points[i]);
                    if (tmpPoint.x * tmpPoint.x + tmpPoint.y * tmpPoint.y > 1600)
                        continue;

                    refertoStart->points.emplace_back(tmpPoint);

                    break;
                }
            }
        }
        // std::cout<<"Tls_lc:"<<Tls_lc<<std::endl;
        // std::cout<<"sum_time: "<<sum_time<<std::endl;
        // std::cout<< "time_increment_:"<<time_increment_<<std::endl;
    }
    return refertoStart;
}

pcl::PointXYZI PointUndistort::PointTransform(const Eigen::Matrix4d &pose, const pcl::PointXYZI &point)
{
    Eigen::Vector4d point_eigen = Eigen::Vector4d(point.x, point.y, point.z, 1);
    Eigen::Vector4d tran_point = pose * point_eigen;

    pcl::PointXYZI tran_point_pcl;
    tran_point_pcl.x = tran_point[0];
    tran_point_pcl.y = tran_point[1];
    tran_point_pcl.z = tran_point[2];
    tran_point_pcl.intensity = point.intensity;
    return tran_point_pcl;
}
