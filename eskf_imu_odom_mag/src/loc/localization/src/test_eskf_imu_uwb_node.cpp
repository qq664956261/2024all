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
bool imu_need_init_ = true;
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
    }
    if (!uwb_inited)
    {
        processUwb();
        uwb_inited = true;
        return;
    }

    // 利用IMU数据进行状态预测
    Predict();
    // 利用UWB数据进行状态更新
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
void processUwb()
{
    Eigen::Vector3d pos;
    Eigen::MatrixXd H(measures_.uwb_.size(), 4);
    Eigen::VectorXd b(measures_.uwb_.size());
    // 取出第一个基站的位置anchor0
    Eigen::Vector3d anchor0 = uwb_position[0];
    std::vector<double> range;

    range.push_back(measures_.uwb_[0]->data_);
    for (int i = 1; i < measures_.uwb_.size(); i++)
    {
        range.push_back(range[i - 1] + measures_.uwb_[i]->data_);
    }
    // 在之后的循环中，函数取出所有测量点的位置anchori
    for (int i = 0; i < measures_.uwb_.size(); i++)
    {
        UWB uwb = *measures_.uwb_[i];
        double range0 = range[i];
        Eigen::Vector3d anchori = uwb_position[measures_.uwb_[i]->idB_];
        H.row(i) = Eigen::Vector4d(anchori.x() - anchor0.x(), anchori.y() - anchor0.y(), anchori.z() - anchor0.z(), range0);
        b[i] = range0 * range0 - anchori.dot(anchori) + anchor0.dot(anchor0);
    }
    H *= -2;
    Eigen::VectorXd x = (H.transpose() * H).inverse() * H.transpose() * b;
    pos = x.head(3);
    Vec3d t = pos;
    // t = T_uwb_base.block<3, 3>(0, 0).inverse() * t + T_uwb_base.block<3, 1>(0, 3);
    Mat3d R;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    if (!uwb_inited)
    {
        Eigen::Matrix4d T_w_uwb;
        T_w_uwb.setIdentity();
        T_w_uwb.block<3, 1>(0, 3) = t;
        T_w_uwb.block<3, 3>(0, 0) = R;
        Eigen::Matrix4d T_w_base = T_w_uwb * T_base_uwb.inverse();
        t = T_w_base.block<3, 1>(0, 3);
    }
    else
    {
        SE3 pose = eskf_.GetNominalSE3();
        Eigen::Matrix4d T_w_base = pose.matrix();
        Eigen::Matrix4d T_w_uwb = T_w_base * T_base_uwb;
        T_w_uwb.block<3, 1>(0, 3) = t;
        T_w_base = T_w_uwb * T_base_uwb.inverse();
        t = T_w_base.block<3, 1>(0, 3);
        // R = T_w_base.block<3, 3>(0, 0);
    }
    SE3 pose_corrected;
    pose_corrected = SE3(R, t);

    eskf_.ObserveSE3(pose_corrected, 1e-1, 1e-1);
}
void corrected()
{
    if (use_opt)
    {
        double para_t[3];
        ceres::Problem problem;
        SE3 pose_predict = eskf_.GetNominalSE3();
        Eigen::Matrix4d T_predict = pose_predict.matrix();
        Eigen::Matrix4d T_predict_uwb = T_predict * T_base_uwb;
        para_t[0] = T_predict_uwb(0, 3);
        para_t[1] = T_predict_uwb(1, 3);
        para_t[2] = T_predict_uwb(2, 3);
        std::cout << "before para_t[0]:" << para_t[0] << std::endl;
        std::cout << "before para_t[1]:" << para_t[1] << std::endl;
        std::cout << "before para_t[2]:" << para_t[2] << std::endl;
        problem.AddParameterBlock(para_t, 3);

        for (int i = 0; i < measures_.uwb_.size(); i++)
        {
            Eigen::Vector3d point_a = uwb_position[measures_.uwb_[i]->idA_];
            Eigen::Vector3d point_b = uwb_position[measures_.uwb_[i]->idB_];
            Eigen::Vector3d point(para_t[0], para_t[1], para_t[2]);
            Eigen::Vector3d diff_i = point - point_a;
            Eigen::Vector3d diff_j = point - point_b;
            double res = diff_i.norm() - diff_j.norm() - measures_.uwb_[i]->data_;
            Eigen::Vector3d diff_ii = point_a - point;
            Eigen::Vector3d diff_jj = point_b - point;
            double res1 = diff_jj.norm() - diff_ii.norm() - measures_.uwb_[i]->data_;
            // std::cout << "res1:" << res1 << std::endl;
            // std::cout << "res:" << res << std::endl;
            // std::cout << "diff_i.norm():" << diff_i.norm() << std::endl;
            // std::cout << "diff_j.norm():" << diff_j.norm() << std::endl;
            // std::cout << "measures_.uwb_[i]->data_:" << measures_.uwb_[i]->data_ << std::endl;
            // std::cout << "res:" << res << std::endl;

            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

            ceres::CostFunction *cost_function = uwbFactor::Create(
                uwb_position[measures_.uwb_[i]->idA_],
                uwb_position[measures_.uwb_[i]->idB_],
                measures_.uwb_[i]->data_);
            problem.AddResidualBlock(cost_function, NULL, para_t);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = 4;
        options.minimizer_progress_to_stdout = false;
        options.max_solver_time_in_seconds = 0.5;
        options.max_num_iterations = 50;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1)
        // {
        //     std::cout << " converge:" << summary.final_cost << std::endl;
        // }
        // else
        // {
        //     std::cout << " not converge :" << summary.final_cost << std::endl;
        // }
        std::cout << "after para_t[0]:" << para_t[0] << std::endl;
        std::cout << "after para_t[1]:" << para_t[1] << std::endl;
        std::cout << "after para_t[2]:" << para_t[2] << std::endl;

        SE3 pose_corrected;
        Eigen::Vector3d t = Eigen::Vector3d(para_t[0], para_t[1], para_t[2]);
        Eigen::Matrix3d R;
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SE3 pose = eskf_.GetNominalSE3();
        Eigen::Matrix4d T_w_base = pose.matrix();
        Eigen::Matrix4d T_w_uwb = T_w_base * T_base_uwb;
        T_w_uwb.block<3, 1>(0, 3) = t;
        T_w_base = T_w_uwb * T_base_uwb.inverse();
        t = T_w_base.block<3, 1>(0, 3);
        // R = T_w_base.block<3, 3>(0, 0);
        pose_corrected = SE3(R, t);
        eskf_.ObserveSE3(pose_corrected, 5e-1, 5e-1);
        SE3 pose_save = eskf_.GetNominalSE3();
        std::ofstream out("/home/zc/uwbFusion.csv", std::ios::app);
        std::ofstream out1("/home/zc/uwb.csv", std::ios::app);
        if (first_csv)
        {
            out << "uwbtime,uwbx,uwby,uwbz" << std::endl;
            out1 << "uwbtime,uwbx,uwby,uwbz" << std::endl;
            out << measures_.uwb_[0]->timestamp_<<","<< pose_save.translation().x() << "," << pose_save.translation().y() << "," << pose_save.translation().z() << std::endl;
            first_csv = false;
            out1 << measures_.uwb_[0]->timestamp_<<","<< pose_corrected.translation().x() << "," << pose_corrected.translation().y() << "," << pose_corrected.translation().z() << std::endl;
        }
        else
        {
            out << measures_.uwb_[0]->timestamp_<<","<< pose_save.translation().x() << "," << pose_save.translation().y() << "," << pose_save.translation().z() << std::endl;
            out1 << measures_.uwb_[0]->timestamp_<<","<< pose_corrected.translation().x() << "," << pose_corrected.translation().y() << "," << pose_corrected.translation().z() << std::endl;
        }

    }
    else
    {
        processUwb();
    }
}
int main(int argc, char *argv[])
{
    T_base_uwb.setIdentity();
    T_base_uwb(0, 3) = -0.01245;
    T_base_uwb(1, 3) = 0.00127;
    T_base_uwb(2, 3) = 0.0908;
    double num;
    uwb_position.push_back(Eigen::Vector3d(-2.4174718660841163, -4.020796001114614, 0.18179046793237785));
    uwb_position.push_back(Eigen::Vector3d(-2.820490062889947, 3.5250373345173456, 2.5874240006860396));
    uwb_position.push_back(Eigen::Vector3d(3.4819322476730066, 3.3050399505325867, 0.15447010668018804));
    uwb_position.push_back(Eigen::Vector3d(3.4507246660737074, -3.7181145718099624, 2.6693201245043428));
    uwb_position.push_back(Eigen::Vector3d(-3.2776160385636026, -3.8689686503275325, 2.67389716671206));
    uwb_position.push_back(Eigen::Vector3d(3.2654739320660124, -3.6510796042048415, 0.1752474453973762));
    uwb_position.push_back(Eigen::Vector3d(3.8321293068358147, 3.652084854209938, 2.624927324400282));
    uwb_position.push_back(Eigen::Vector3d(-2.7227724068629255, 3.21907986264268, 0.15829414514009574));

    sync_ = std::make_shared<MessageSync>([](const MeasureGroup &m)
                                          { ProcessMeasurements(m); });
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_eskf_imu_uwb_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_uwb;
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu_data", 1000, boost::bind(&MessageSync::ProcessIMU, sync_, _1));
    sub_uwb = nh.subscribe<cf_msgs::Tdoa>("/tdoa_data", 1000, boost::bind(&MessageSync::ProcessUwb, sync_, _1));

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
        // std::cout << "pose:" << pose.translation().transpose() << std::endl;
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
