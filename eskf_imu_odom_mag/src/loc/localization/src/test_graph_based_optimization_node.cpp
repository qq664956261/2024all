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
#include "localization/factor/uwbFactorGraph.h"
#include "localization/factor/uwbFactor.h"
#include "localization/factor/lidarFactor.h"
#include "localization/sync/measure_sync.h"
#include "localization/imu_init/static_imu_init.h"
#include "localization/pre_integrator/imu_factor.h"
#include "localization/pre_integrator/pose_local_parameterization.h"
using namespace localization;
using namespace sad;
void ProcessMeasurements(const MeasureGroup &meas);
void TryInitIMU();
void processUwb();
void optimization();
void double2vector();
void vector2double();
void slideWindow();
MeasureGroup measures_;

StaticIMUInit imu_init_;                      // IMU初始化器
std::shared_ptr<MessageSync> sync_ = nullptr; // 消息同步器
bool imu_need_init_ = true;
bool uwb_inited = false;
std::vector<NavStated> imu_states_; // ESKF预测期间的状态
std::vector<Eigen::Vector3d> uwb_position;
Eigen::Matrix4d T_base_uwb;
bool use_opt = true;
IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
int frame_count;
Vector3d acc_0, gyr_0;
double last_imu_t = 0;
Vector3d g;
std::deque<UWBPtr> uwb_ptrs[WINDOW_SIZE + 1];
Vector3d Ps[(WINDOW_SIZE + 1)];
Vector3d Vs[(WINDOW_SIZE + 1)];
Vector3d Times[(WINDOW_SIZE + 1)];
Matrix3d Rs[(WINDOW_SIZE + 1)];
Vector3d Bas[(WINDOW_SIZE + 1)];
Vector3d Bgs[(WINDOW_SIZE + 1)];
bool first_csv = true;
double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
std::vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
std::vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

void ProcessMeasurements(const MeasureGroup &meas)
{

    measures_ = meas;
    if (imu_need_init_)
    {
        TryInitIMU();
    }
    if (!uwb_inited)
    {
        if (frame_count == WINDOW_SIZE)
        {
            uwb_inited = true;
        }
        std::cout << "222" << std::endl;
        processUwb();

        return;
    }

    optimization();
}
void optimization()
{
    vector2double();
    std::cout << "optimization before" << Rs[WINDOW_SIZE] << std::endl;
    // 借助ceres进行非线性优化
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    // Step 1 定义待优化的参数块，类似g2o的顶点
    // 参数块 1： 滑窗中位姿包括位置和姿态，共11帧
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        // 由于姿态不满足正常的加法，也就是李群上没有加法，因此需要自己定义他的加法
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    problem.SetParameterBlockConstant(para_Pose[0]);
    problem.SetParameterBlockConstant(para_SpeedBias[0]);
    for (auto imu : measures_.imu_)
    {
        double dt = imu->timestamp_ - last_imu_t;
        last_imu_t = imu->timestamp_;
        Eigen::Vector3d linear_acceleration = imu->acce_;
        Eigen::Vector3d angular_velocity = imu->gyro_;

        if (!pre_integrations[WINDOW_SIZE])
        {
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{linear_acceleration, angular_velocity, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
        }
        pre_integrations[WINDOW_SIZE]->push_back(dt, linear_acceleration, angular_velocity);

        // 保存传感器数据
        dt_buf[WINDOW_SIZE].push_back(dt);
        linear_acceleration_buf[WINDOW_SIZE].push_back(linear_acceleration);
        angular_velocity_buf[WINDOW_SIZE].push_back(angular_velocity);

        Vector3d un_acc_0 = Rs[WINDOW_SIZE] * (acc_0 - Bas[WINDOW_SIZE]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[WINDOW_SIZE];
        Rs[WINDOW_SIZE] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();

        Vector3d un_acc_1 = Rs[WINDOW_SIZE] * (linear_acceleration - Bas[WINDOW_SIZE]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[WINDOW_SIZE] += dt * Vs[WINDOW_SIZE] + 0.5 * dt * dt * un_acc;
        Vs[WINDOW_SIZE] += dt * un_acc;
        std::cout << "Vs[WINDOW_SIZE]:" << Vs[WINDOW_SIZE] << std::endl;
        std::cout << "Vs[WINDOW_SIZE-1]:" << Vs[WINDOW_SIZE - 1] << std::endl;
        std::cout << "dt:" << dt << std::endl;
        std::cout << "un_acc:" << un_acc << std::endl;
        std::cout << "ua_0:" << un_acc_0 << std::endl;
        std::cout << "ua_1:" << un_acc_1 << std::endl;
        std::cout << "acc_0:" << acc_0 << std::endl;
        std::cout << "Rs[WINDOW_SIZE] * acc_0 :" << Rs[WINDOW_SIZE] * acc_0 << std::endl;
        std::cout << "Rs[WINDOW_SIZE-1]:" << Rs[WINDOW_SIZE - 1] << std::endl;
        std::cout << "Rs[WINDOW_SIZE]:" << Rs[WINDOW_SIZE] << std::endl;
        std::cout << "acc_0.norm():" << acc_0.norm() << std::endl;
        std::cout << "(Rs[WINDOW_SIZE] * acc_0).norm():" << (Rs[WINDOW_SIZE] * acc_0).norm() << std::endl;
        std::cout << "gyr_0:" << gyr_0 << std::endl;
        std::cout << "angular_velocity:" << angular_velocity << std::endl;
        std::cout << "un_gyr:" << un_gyr << std::endl;
        std::cout << "un_gyr * dt:" << un_gyr * dt << std::endl;
        std::cout << "Utility::deltaQ(un_gyr * dt).toRotationMatrix():"
                  << Utility::deltaQ(un_gyr * dt).toRotationMatrix() << std::endl;


        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    // std::cout << "1111Ps[WINDOW_SIZE]:" << Ps[WINDOW_SIZE] << std::endl;
    // imu预积分的约束
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        // 时间过长这个约束就不可信了
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    // for (int i = 0; i < WINDOW_SIZE; i++)
    // {
    //     Eigen::Vector3d para_t_constraint;
    //     Eigen::Vector4f para_q_constraint;
    //     Eigen::Matrix3d temp_R2 = Rs[i + 1];
    //     Eigen::Quaterniond temp_q_2(temp_R2);
    //     Eigen::Vector3d temp_t_2 = Ps[i + 1];
    //     Eigen::Matrix3d temp_R1 = Rs[i];
    //     Eigen::Quaterniond temp_q_1(temp_R1);
    //     Eigen::Vector3d temp_t_1 = Ps[i];

    //     Eigen::Matrix3d R_constraint;
    //     Eigen::Vector3d t_constraint;

    //     R_constraint = temp_q_2.toRotationMatrix().inverse() * temp_q_1.toRotationMatrix();
    //     t_constraint = temp_q_2.toRotationMatrix().inverse() * (temp_t_1 - temp_t_2);

    //     Eigen::Quaterniond q_constraint(R_constraint);

    //     para_t_constraint = t_constraint;
    //     para_q_constraint[0] = q_constraint.x();
    //     para_q_constraint[1] = q_constraint.y();
    //     para_q_constraint[2] = q_constraint.z();
    //     para_q_constraint[3] = q_constraint.w();
    //     ceres::CostFunction *cost_function = consecutivePose::Create(
    //         para_q_constraint,
    //         para_t_constraint);
    //     problem.AddResidualBlock(cost_function, NULL, para_Pose[i], para_Pose[i + 1]);
    // }
    std::ofstream out("/home/zc/uwbGraphFusion.csv", std::ios::app);
    uwb_ptrs[WINDOW_SIZE] = measures_.uwb_;
    Eigen::Vector3d time(measures_.uwb_[0]->timestamp_, measures_.uwb_[0]->timestamp_, measures_.uwb_[0]->timestamp_);
    Times[WINDOW_SIZE] = time;
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        for (int j = 0; j < uwb_ptrs[i].size(); j++)
        {
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

            ceres::CostFunction *cost_function = uwbFactorGraph::Create(
                uwb_position[uwb_ptrs[i][j]->idA_],
                uwb_position[uwb_ptrs[i][j]->idB_],
                T_base_uwb,
                uwb_ptrs[i][j]->data_);
            problem.AddResidualBlock(cost_function, NULL, para_Pose[i]);
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = false;
    options.max_solver_time_in_seconds = 5;
    options.max_num_iterations = 500;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "para_Pose[i][0]:" << para_Pose[WINDOW_SIZE][0] << std::endl;
    std::cout << "para_Pose[i][1]:" << para_Pose[WINDOW_SIZE][1] << std::endl;
    std::cout << "para_Pose[i][2]:" << para_Pose[WINDOW_SIZE][2] << std::endl;
    std::cout << "para_Pose[i][3]:" << para_Pose[WINDOW_SIZE][3] << std::endl;
    std::cout << "para_Pose[i][4]:" << para_Pose[WINDOW_SIZE][4] << std::endl;
    std::cout << "para_Pose[i][5]:" << para_Pose[WINDOW_SIZE][5] << std::endl;
    std::cout << "para_Pose[i][6]:" << para_Pose[WINDOW_SIZE][6] << std::endl;
    std::cout << "para_SpeedBias[i][0]:" << para_SpeedBias[WINDOW_SIZE][0] << std::endl;
    std::cout << "para_SpeedBias[i][1]:" << para_SpeedBias[WINDOW_SIZE][1] << std::endl;
    std::cout << "para_SpeedBias[i][2]:" << para_SpeedBias[WINDOW_SIZE][2] << std::endl;
    std::cout << "para_SpeedBias[i][3]:" << para_SpeedBias[WINDOW_SIZE][3] << std::endl;
    std::cout << "para_SpeedBias[i][4]:" << para_SpeedBias[WINDOW_SIZE][4] << std::endl;
    std::cout << "para_SpeedBias[i][5]:" << para_SpeedBias[WINDOW_SIZE][5] << std::endl;
    std::cout << "para_SpeedBias[i][6]:" << para_SpeedBias[WINDOW_SIZE][6] << std::endl;
    std::cout << "para_SpeedBias[i][7]:" << para_SpeedBias[WINDOW_SIZE][7] << std::endl;
    std::cout << "para_SpeedBias[i][8]:" << para_SpeedBias[WINDOW_SIZE][8] << std::endl;

    if (first_csv)
    {
        out << "uwbtime,uwbx,uwby,uwbz" << std::endl;
        out << measures_.uwb_[0]->timestamp_ << "," << para_Pose[WINDOW_SIZE][0] << "," << para_Pose[WINDOW_SIZE][1] << "," << para_Pose[WINDOW_SIZE][2] << std::endl;
        first_csv = false;
    }
    else
    {
        out << measures_.uwb_[0]->timestamp_ << "," << para_Pose[WINDOW_SIZE][0] << "," << para_Pose[WINDOW_SIZE][1] << "," << para_Pose[WINDOW_SIZE][2] << std::endl;
    }

    double2vector();
    std::cout << "after Rs[WINDOW_SIZE]:" << Rs[WINDOW_SIZE] << std::endl;
    slideWindow();
}
void slideWindow()
{
    if (frame_count == WINDOW_SIZE)
    {
        std::cout << "slideWindow" << std::endl;
        // 一帧一帧交换过去
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            Rs[i].swap(Rs[i + 1]);

            std::swap(pre_integrations[i], pre_integrations[i + 1]);

            dt_buf[i].swap(dt_buf[i + 1]);
            linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
            angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

            uwb_ptrs[i].swap(uwb_ptrs[i + 1]);
            Times[i].swap(Times[i + 1]);
            Ps[i].swap(Ps[i + 1]);
            Vs[i].swap(Vs[i + 1]);
            Bas[i].swap(Bas[i + 1]);
            Bgs[i].swap(Bgs[i + 1]);
        }
        // 最后一帧的状态量赋上当前值，最为初始值

        Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
        Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
        Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
        Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
        Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
        // 预积分量就得置零
        delete pre_integrations[WINDOW_SIZE];
        pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};
        // buffer清空，等待新的数据来填
        dt_buf[WINDOW_SIZE].clear();
        linear_acceleration_buf[WINDOW_SIZE].clear();
        angular_velocity_buf[WINDOW_SIZE].clear();
    }
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
        // eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;
        acc_0 = measures_.imu_.back()->acce_;
        gyr_0 = measures_.imu_.back()->gyro_;
        Bas[frame_count] = imu_init_.GetInitBa();
        Bgs[frame_count] = imu_init_.GetInitBg();
        std::cout << "Bas[frame_count]:" << Bas[frame_count] << std::endl;
        std::cout << "Bgs[frame_count]:" << Bgs[frame_count] << std::endl;

        // g = imu_init_.GetGravity();
        g << 0, 0, 9.81;
        last_imu_t = measures_.imu_.back()->timestamp_;

        if (!pre_integrations[frame_count])
        {
            pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
        }

        LOG(INFO) << "IMU初始化成功";
    }
}

void processUwb()
{
    Eigen::Vector3d pos;
    // Eigen::MatrixXd H(measures_.uwb_.size(), 4);
    // Eigen::VectorXd b(measures_.uwb_.size());
    // // 取出第一个基站的位置anchor0
    // Eigen::Vector3d anchor0 = uwb_position[0];
    // std::vector<double> range;

    // range.push_back(measures_.uwb_[0]->data_);
    // for (int i = 1; i < measures_.uwb_.size(); i++)
    // {
    //     range.push_back(range[i - 1] + measures_.uwb_[i]->data_);
    // }
    // // 在之后的循环中，函数取出所有测量点的位置anchori
    // for (int i = 0; i < measures_.uwb_.size(); i++)
    // {
    //     UWB uwb = *measures_.uwb_[i];
    //     double range0 = range[i];
    //     Eigen::Vector3d anchori = uwb_position[measures_.uwb_[i]->idB_];
    //     H.row(i) = Eigen::Vector4d(anchori.x() - anchor0.x(), anchori.y() - anchor0.y(), anchori.z() - anchor0.z(), range0);
    //     b[i] = range0 * range0 - anchori.dot(anchori) + anchor0.dot(anchor0);
    // }
    // H *= -2;
    // Eigen::VectorXd x = (H.transpose() * H).inverse() * H.transpose() * b;
    // pos = x.head(3);
    double para_t[3];
    ceres::Problem problem;
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
    pos = Eigen::Vector3d(para_t[0], para_t[1], para_t[2]);
    std::cout << "pos:" << pos << std::endl;

    Vec3d t = pos;
    // t = T_uwb_base.block<3, 3>(0, 0).inverse() * t + T_uwb_base.block<3, 1>(0, 3);
    Eigen::Matrix4d T_w_uwb;
    T_w_uwb.setIdentity();
    T_w_uwb.block<3, 1>(0, 3) = t;
    Eigen::Matrix4d T_w_base = T_w_uwb * T_base_uwb.inverse();
    t = T_w_base.block<3, 1>(0, 3);
    Mat3d R;
    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    SE3 pose_corrected;
    pose_corrected = SE3(R, t);
    Rs[frame_count] = R;
    Ps[frame_count] = t;
    uwb_ptrs[frame_count] = measures_.uwb_;
    Eigen::Vector3d time(measures_.uwb_[0]->timestamp_, measures_.uwb_[0]->timestamp_, measures_.uwb_[0]->timestamp_);
    Times[frame_count] = time;

    if (frame_count != 0)
    {
        bool first_imu = true;
        for (auto imu : measures_.imu_)
        {
            double dt = imu->timestamp_ - last_imu_t;
            last_imu_t = imu->timestamp_;
            Eigen::Vector3d linear_acceleration = imu->acce_;
            Eigen::Vector3d angular_velocity = imu->gyro_;
            if (!pre_integrations[frame_count])
            {
                pre_integrations[frame_count] = new IntegrationBase{linear_acceleration, angular_velocity, Bas[frame_count], Bgs[frame_count]};
            }
            pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

            // 保存传感器数据
            dt_buf[frame_count].push_back(dt);
            linear_acceleration_buf[frame_count].push_back(linear_acceleration);
            angular_velocity_buf[frame_count].push_back(angular_velocity);
            // 又是一个中值积分，更新滑窗中状态量，本质是给非线性优化提供可信的初始值
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            // Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            // Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;

            if (first_imu)
            {
                first_imu = false;
                Vs[j] = Vs[j - 1] + dt * un_acc;
                Rs[j] = Rs[j - 1] * Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            }
            else
            {
                Vs[j] += dt * un_acc;
                Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            }
            acc_0 = linear_acceleration;
            gyr_0 = angular_velocity;
            std::cout << "acc_0:" << acc_0 << std::endl;
            std::cout << "linear_acceleration:" << linear_acceleration << std::endl;
            std::cout << "un_acc:" << un_acc << std::endl;
            std::cout << "dt:" << dt << std::endl;
            std::cout << "dt * un_acc:" << dt * un_acc << std::endl;
            std::cout << "Vs[j]:" << Vs[j] << std::endl;
            std::cout << "Rs[j]:" << Rs[j] << std::endl;
            std::cout << "Ps[j]:" << Ps[j] << std::endl;

        }
    }

    if (frame_count < WINDOW_SIZE)
    {
        frame_count++;
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE - 1; i++)
        {
            Vs[i + 1] = (Ps[i + 1] - Ps[i]) / (Times[i + 1][0] - Times[i][0]);
        }
        Vs[0] = Vs[1];
    }
}

void vector2double()
{
    // KF的位姿
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
}

void double2vector()
{

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = Vector3d(para_Pose[i][0],
                         para_Pose[i][1],
                         para_Pose[i][2]);

        Vs[i] = Vector3d(para_SpeedBias[i][0],
                         para_SpeedBias[i][1],
                         para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    for (int i = 0; i <= WINDOW_SIZE - 1; i++)
    {
        Vs[i + 1] = (Ps[i + 1] - Ps[i]) / (Times[i + 1][0] - Times[i][0]);
        // std::cout << "Vs[i + 1]:" << Vs[i + 1] << std::endl;
        // std::cout << "Ps[i + 1]:" << Ps[i + 1] << std::endl;
    }
    Vs[0] = Vs[1];
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

    ros::init(argc, argv, "test_graph_based_optimization_node");
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

        odometry_matrix(0, 3) = para_Pose[WINDOW_SIZE][0];
        odometry_matrix(1, 3) = para_Pose[WINDOW_SIZE][1];
        odometry_matrix(2, 3) = para_Pose[WINDOW_SIZE][2];

        odom_pub_ptr->Publish(odometry_matrix);
        num++;
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
