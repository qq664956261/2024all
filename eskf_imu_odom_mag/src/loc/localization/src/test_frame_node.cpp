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
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
using namespace localization;
using namespace sad;
int main(int argc, char *argv[])
{
    std::vector<Eigen::Vector3d> uwb_position;
    uwb_position.push_back(Eigen::Vector3d(-2.4174718660841163, -4.020796001114614, 0.18179046793237785));
    uwb_position.push_back(Eigen::Vector3d(-2.820490062889947, 3.5250373345173456, 2.5874240006860396));
    uwb_position.push_back(Eigen::Vector3d(3.4819322476730066, 3.3050399505325867, 0.15447010668018804));
    uwb_position.push_back(Eigen::Vector3d(3.4507246660737074, -3.7181145718099624, 2.6693201245043428));
    uwb_position.push_back(Eigen::Vector3d(-3.2776160385636026, -3.8689686503275325, 2.67389716671206));
    uwb_position.push_back(Eigen::Vector3d(3.2654739320660124, -3.6510796042048415, 0.1752474453973762));
    uwb_position.push_back(Eigen::Vector3d(3.8321293068358147, 3.652084854209938, 2.624927324400282));
    uwb_position.push_back(Eigen::Vector3d(-2.7227724068629255, 3.21907986264268, 0.15829414514009574));
    sad::ESKFD eskf;
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/imu_data", 1000000);
    std::shared_ptr<UWBSubscriber> uwb_sub_ptr = std::make_shared<UWBSubscriber>(nh, "/tdoa_data", 1000000);
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::deque<CloudData> cloud_data_buff;
    std::deque<IMU> imu_data_buff;
    std::deque<UWB> uwb_data_buff;
    std::deque<UWB> uwb_data_buff_calculation;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool start_time = false;
    double start_time_stamp = 0.0;
            
    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        uwb_sub_ptr->ParseData(uwb_data_buff);

        while (uwb_data_buff.size() > 0)
        {
            if (!start_time)
            {
                UWB uwb = uwb_data_buff.front();
                if(uwb.idA_ == 0)
                {
                    start_time = true;
                    uwb_data_buff_calculation.push_back(uwb_data_buff.front());
                    uwb_data_buff.pop_front();

                }
                else{
                    uwb_data_buff.pop_front();

                }
                continue;
            }
            UWB cur_uwb = uwb_data_buff.front();

            if (cur_uwb.idA_ > 0 && cur_uwb.idA_ < 8)
            {
                uwb_data_buff_calculation.push_back(uwb_data_buff.front());
                uwb_data_buff.pop_front();
                std::cout << "uwb_data_buff_calculation.size():" << uwb_data_buff_calculation.size() << std::endl;
            }
            else
            {
                std::cout << "calculate" << std::endl;
                // double para_t[3];
                // ceres::Problem problem;
                // problem.AddParameterBlock(para_t, 3);
                // for (int i = 0; i < uwb_data_buff_calculation.size(); i++)
                // {

                //     ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

                //     ceres::CostFunction *cost_function = uwbFactor::Create(
                //         uwb_position[uwb_data_buff_calculation[i].idA_],
                //         uwb_position[uwb_data_buff_calculation[i].idB_],
                //         uwb_data_buff_calculation[i].data_);
                //     problem.AddResidualBlock(cost_function, NULL, para_t);
                // }
                // std::cout << "para_t[0]:" << para_t[0] << std::endl;
                // std::cout << "para_t[1]:" << para_t[1] << std::endl;
                // std::cout << "para_t[2]:" << para_t[2] << std::endl;
                Eigen::Vector3d pos;
                Eigen::MatrixXd H(uwb_data_buff_calculation.size(), 4);
                Eigen::VectorXd b(uwb_data_buff_calculation.size());
                // 取出第一个基站的位置anchor0
                Eigen::Vector3d anchor0 = uwb_position[0];
                std::vector<double> range;

                range.push_back(uwb_data_buff_calculation[0].data_);
                for (int i = 1; i < uwb_data_buff_calculation.size(); i++)
                {
                    range.push_back(range[i - 1] + uwb_data_buff_calculation[i].data_);
                }
                // 在之后的循环中，函数取出所有测量点的位置anchori
                for (int i = 0; i < uwb_data_buff_calculation.size(); i++)
                {
                    std::cout<<"uwb_data_buff_calculation[i].idA_:"<<uwb_data_buff_calculation[i].idA_<<std::endl;
                    std::cout<<"uwb_data_buff_calculation[i].idB_:"<<uwb_data_buff_calculation[i].idB_<<std::endl;
                    UWB uwb = uwb_data_buff_calculation[i];
                    double range0 = range[i];
                    Eigen::Vector3d anchori = uwb_position[uwb_data_buff_calculation[i].idB_];
                    H.row(i) = Eigen::Vector4d(anchori.x() - anchor0.x(), anchori.y() - anchor0.y(), anchori.z() - anchor0.z(), range0);
                    b[i] = range0 * range0 - anchori.dot(anchori) + anchor0.dot(anchor0);
                }
                H *= -2;
                Eigen::VectorXd x = (H.transpose() * H).inverse() * H.transpose() * b;
                pos = x.head(3);
                odometry_matrix(0,3) = pos.x(); 
                odometry_matrix(1,3) = pos.y();
                odometry_matrix(2,3) = pos.z();
                std::cout << "pos:" << pos << std::endl;
                uwb_data_buff_calculation.clear();
                start_time = false;
                start_time_stamp = 0.0;
            }

            

            odom_pub_ptr->Publish(odometry_matrix);
        }

        rate.sleep();
    }

    return 0;
}