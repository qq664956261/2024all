
// create by zhang cheng 2024.4.17
#ifndef MEASURE_SYNC_H
#define MEASURE_SYNC_H

#include "localization/common/imu.h"
#include "localization/common/uwb.h"
#include "localization/common/odom.h"
#include "cf_msgs/Tdoa.h"
#include "sensor_msgs/Imu.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Mag.h"
#include <glog/logging.h>
#include <deque>
#include <mutex>
namespace sad
{

    ///  预测数据(imu)与观测数据同步
    struct MeasureGroup
    {
        MeasureGroup() {};

        double observation_time_ = 0; // 观测数据时间
        sad::UWB observation_uwb_;    // 观测类型数据
        sad::Odom observation_odom_;
        std::deque<IMUPtr> imu_; // 上一时时刻到现在的IMU读数
        std::deque<UWBPtr> uwb_; // 上一时时刻到现在的UWB读数
        std::deque<OdomPtr> odom_;
        double yaw_;
    };

    /**
     * 将观测数据和预测数据(imu)同步
     */
    class MessageSync
    {
    public:
        using Callback = std::function<void(const MeasureGroup &)>;

        MessageSync(Callback cb) : callback_(cb) {}

        /// 初始化
        void Init(const std::string &yaml);

        /// 处理IMU数据
        void ProcessIMU(const hj_interface::Imu::ConstPtr &imu_msg_ptr)
        {

            IMUPtr imu = std::make_shared<IMU>();
            // imu->timestamp_ = imu_msg_ptr->header.stamp.toSec();
            imu->timestamp_ = imu_msg_ptr->custom_time.toSec();
            //std::cout<<imu->timestamp_<<std::endl;
            Eigen::AngleAxisd rotation_vector(1.57, Eigen::Vector3d(0, 0, 1)); 
            Eigen::Matrix3d R = rotation_vector.matrix();
            imu->acce_ = Eigen::Vector3d(imu_msg_ptr->accel_x / 1000.0 * 9.81,
                                         imu_msg_ptr->accel_y / 1000.0 * 9.81,
                                         imu_msg_ptr->accel_z / 1000.0 * 9.81);

            imu->gyro_ =  Eigen::Vector3d(imu_msg_ptr->gyro_x / 100.0 * M_PI / 180.0,
                                         imu_msg_ptr->gyro_y / 100.0 * M_PI / 180.0,
                                         imu_msg_ptr->gyro_z / 100.0 * M_PI / 180.0);


            
        
            imu->roll = imu_msg_ptr->roll / 100.0 * M_PI / 180.0;
            imu->pitch = imu_msg_ptr->pitch / 100.0 * M_PI / 180.0; 
            imu->yaw = imu_msg_ptr->yaw / 100.0 * M_PI / 180.0;

            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(imu->pitch,Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(imu->roll,Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(imu->yaw,Eigen::Vector3d::UnitZ())); 
            Eigen::Matrix3d rotation_matrix=yawAngle.toRotationMatrix()*pitchAngle.toRotationMatrix()*rollAngle.toRotationMatrix();
            rotation_matrix = R * rotation_matrix;
            Eigen::Vector3d acc_r = rotation_matrix * imu->acce_;
            
            if(std::fabs(acc_r[0]) > 2 || std::fabs(acc_r[1]) > 2)
            {
                std::cout<<"imu->acce_:"<<imu->acce_<<std::endl;
                std::cout<<"imu->roll:"<<imu->roll<<std::endl;
                std::cout<<"imu->pitch:"<<imu->pitch<<std::endl;
                std::cout<<"acc_r_:"<<acc_r<<std::endl;
            }
                        
            if(std::fabs(acc_r[2]) > 11.5)
            {
                std::cout<<"imu->acce_:"<<imu->acce_<<std::endl;
                std::cout<<"imu->roll:"<<imu->roll<<std::endl;
                std::cout<<"imu->pitch:"<<imu->pitch<<std::endl;
                std::cout<<"acc_r_:"<<acc_r<<std::endl;
            }
            // imu->orientation.x = imu_msg_ptr->orientation.x;
            // imu->orientation.y = imu_msg_ptr->orientation.y;
            // imu->orientation.z = imu_msg_ptr->orientation.z;
            // imu->orientation.w = imu_msg_ptr->orientation.w;
            double timestamp = imu->timestamp_;
            if (timestamp < last_timestamp_imu_)
            {
                LOG(WARNING) << "imu loop back, clear buffer";
                imu_buffer_.clear();
            }


            last_timestamp_imu_ = timestamp;

            imu_buffer_.emplace_back(imu);
            mag_yaw_.push_back(imu_msg_ptr->yaw / 100.0 * M_PI / 180.0);
        }

        /**
         * 处理观测uwb数据
         * @param
         */
        // void ProcessUwb(const cf_msgs::Tdoa::ConstPtr& msg) {
        //     UWBPtr uwb = std::make_shared<UWB>();
        //     uwb->timestamp_ = msg->header.stamp.toSec();
        //     uwb->idA_ = msg->idA;
        //     uwb->idB_ = msg->idB;
        //     uwb->data_ = msg->data;
        //     if (uwb->timestamp_ < last_timestamp_uwb_) {
        //         LOG(ERROR) << "lidar loop back, clear buffer";
        //         uwb_buffer_.clear();
        //     }

        //     uwb_buffer_.push_back(uwb);
        //     time_buffer_.push_back(uwb->timestamp_);
        //     last_timestamp_uwb_ = uwb->timestamp_;
        //     Sync();

        // }

        /**
         * 处理观测odom数据
         * @param
         */
        void ProcessOdom(const hj_interface::Encoder::ConstPtr &msg)
        {
            double encoder_scale = 1e-3;
            OdomPtr odom = std::make_shared<sad::Odom>();
            odom->timestamp_ = msg->custom_time.toSec();
            odom->left_pulse_ = msg->left_msg * encoder_scale;
            odom->right_pulse_ = msg->right_msg * encoder_scale;

            if (odom->timestamp_ < last_timestamp_odom_)
            {
                LOG(ERROR) << "odom loop back, clear buffer";
                odom_buffer_.clear();
            }

            odom_buffer_.push_back(odom);
            time_buffer_.push_back(odom->timestamp_);
            last_timestamp_odom_ = odom->timestamp_;
            Sync();
        }

        void ProcessMag(const hj_interface::Mag::ConstPtr &msg)
        {
            double magnetic_offset_x = -6.5;
            double magnetic_offset_y = 6.0;
            double magnetic_offset_z = 3.0;
            float Hx, Hy;
            //  short old_mag[2] = {0};

            float Vxoff = magnetic_offset_x;
            float Vyoff = magnetic_offset_y;
            //  short cali_flag = 1;

            float result_yaw = 0.0f;

            Hx = msg->mag_x - Vxoff;
            Hy = msg->mag_y - Vyoff;

            result_yaw = atan2(Hy, Hx); //* 180.0f / 3.1416f;
            // std::cout<<"result_yaw:"<<result_yaw<<std::endl;
            // mag_yaw_.push_back(result_yaw);
        }
        std::mutex mtx_imu;
        std::mutex mtx_uwb;
        std::mutex mtx_sync;

    private:
        /// 尝试同步预测数据imu与观测数据uwb，成功时返回true
        bool Sync();

        Callback callback_; // 同步数据后的回调函数
        // std::deque<UWBPtr> uwb_buffer_;                 // 观测uwb数据缓冲
        std::deque<IMUPtr> imu_buffer_; // imu数据缓冲
        std::deque<OdomPtr> odom_buffer_;
        std::deque<double> mag_yaw_;
        double last_timestamp_imu_ = -1.0; // 最近imu时间
        double last_timestamp_uwb_ = 0;    // 最近uwb时间
        double last_timestamp_odom_ = 0;
        std::deque<double> time_buffer_;
        bool uwb_pushed_ = false;
        bool odom_pushed_ = false;
        MeasureGroup measures_; // 同步后的数据
        double uwb_end_time_ = 0;
        double odom_end_time_ = 0;
        bool first_imu = true;
        IMUPtr last_imu;
    };

} // namespace sad

#endif // MEASURE_SYNC_H
