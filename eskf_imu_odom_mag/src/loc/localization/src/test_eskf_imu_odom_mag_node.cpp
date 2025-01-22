/*
 * @Description:
 * @Author: zhang cheng
 * @Date: 2024-04-11 15:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "localization/global_defination/global_defination.h"
#include "localization/subscriber/cloud_subscriber.hpp"
#include "localization/subscriber/imu_subscriber.hpp"
#include "localization/tf_listener/tf_listener.hpp"
#include "localization/publisher/cloud_publisher.hpp"
#include "localization/publisher/odometry_publisher.hpp"
#include "localization/eskf/eskf.hpp"
#include "localization/subscriber/uwb_subscriber.hpp"
#include "localization/factor/uwbFactor.h"
#include "localization/sync/measure_sync.h"
#include "localization/imu_init/static_imu_init.h"

using namespace localization;
using namespace sad;
void ProcessMeasurements(const MeasureGroup &meas);
void TryInitIMU();
void Predict();
void processUwb();
void corrected();
MeasureGroup measures_;
sad::ESKFD eskf_;
StaticIMUInit imu_init_;                      // IMU初始化器
std::shared_ptr<MessageSync> sync_ = nullptr; // 消息同步器
bool imu_need_init_ = false;
bool mag_need_init_ = true;
bool uwb_inited = false;
std::vector<NavStated> imu_states_; // ESKF预测期间的状态
std::vector<Eigen::Vector3d> uwb_position;
Eigen::Matrix4d T_base_uwb;
bool use_opt = true;
bool first_csv = true;
void ProcessMeasurements(const MeasureGroup &meas)
{
    measures_ = meas;
    if (imu_need_init_)
    {
        TryInitIMU();
        return;
    }

    if (mag_need_init_)
    {
        double yaw = measures_.yaw_;
        //Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1)); // 沿 Z 轴旋转 45 度Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1)); // 沿 Z 轴旋转 45 度
        // cout.precision(3);
        // cout << "rotation matrix =\n"
        //      << rotation_vector.matrix() << endl;
        SO3 R = SO3::exp(Vec3d(0,0,yaw));
        eskf_.SetR(R);
        mag_need_init_ = false;

    }

    // 利用IMU数据进行状态预测
    Predict();
    // 利用Odom数据进行状态更新
    corrected();
}
void TryInitIMU()
{
    for (auto imu : measures_.imu_)
    {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess())
    {
        // 读取初始零偏，设置ESKF
        sad::ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
    }
    else
    {
        LOG(INFO) << "IMU初始化失败";
    }
}
void Predict()
{
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());
    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_)
    {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
    }
}

void corrected()
{

    eskf_.ObserveWheelSpeed(measures_.observation_odom_);

    SE3 pose_corrected;
    Eigen::AngleAxisd rotation_vector(measures_.yaw_, Eigen::Vector3d(0, 0, 1)); 
    Eigen::Matrix3d R = rotation_vector.matrix();

    SE3 pose = eskf_.GetNominalSE3();
    Eigen::Vector3d t = pose.translation();
    pose_corrected = SE3(R, t);
    eskf_.ObserveSE3(pose_corrected, 5e-2, 5e-2);
}
int main(int argc, char *argv[])
{

    double num;

    sync_ = std::make_shared<MessageSync>([](const MeasureGroup &m)
                                          { ProcessMeasurements(m); });
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_eskf_imu_uwb_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_uwb;
    ros::Subscriber sub_mag;
    sub_imu = nh.subscribe<hj_interface::Imu>("/imu_chatter", 1000, boost::bind(&MessageSync::ProcessIMU, sync_, _1));
    sub_uwb = nh.subscribe<hj_interface::Encoder>("/motor_chatter", 1000, boost::bind(&MessageSync::ProcessOdom, sync_, _1));
    sub_mag = nh.subscribe<hj_interface::Mag>("/mag_chatter", 1000, boost::bind(&MessageSync::ProcessMag, sync_, _1));
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool start_time = false;
    double start_time_stamp = 0.0;

    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
    ros::Rate rate(100);
    while (ros::ok())
    {
        SE3 pose = eskf_.GetNominalSE3();
        odometry_matrix(0, 3) = pose.translation().x();
        odometry_matrix(1, 3) = pose.translation().y();
        odometry_matrix(2, 3) = pose.translation().z();


        // if(num%20==0)
        odom_pub_ptr->Publish(odometry_matrix);
        num++;
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
