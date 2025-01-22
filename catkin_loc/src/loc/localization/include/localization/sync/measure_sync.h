
//create by zhang cheng 2024.4.17
#ifndef MEASURE_SYNC_H
#define MEASURE_SYNC_H


#include "localization/common/imu.h"
#include "localization/common/uwb.h"
#include "cf_msgs/Tdoa.h"
#include "sensor_msgs/Imu.h"
#include <glog/logging.h>
#include <deque>
#include <mutex>
namespace sad {

///  预测数据(imu)与观测数据同步
struct MeasureGroup {
    MeasureGroup() {  };

    double observation_time_ = 0;   // 观测数据时间
    sad::UWB observation_uwb_;  // 观测类型数据
    std::deque<IMUPtr> imu_;        // 上一时时刻到现在的IMU读数
    std::deque<UWBPtr> uwb_;        // 上一时时刻到现在的UWB读数
};

/**
 * 将观测数据和预测数据(imu)同步
 */
class MessageSync {
   public:
    using Callback = std::function<void(const MeasureGroup &)>;

    MessageSync(Callback cb) : callback_(cb) {}

    /// 初始化
    void Init(const std::string &yaml);

    /// 处理IMU数据
    void ProcessIMU(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
        IMUPtr imu = std::make_shared<IMU>();
        imu->timestamp_ = imu_msg_ptr->header.stamp.toSec();
        imu->acce_ = Eigen::Vector3d(imu_msg_ptr->linear_acceleration.x * 9.81,
                                                     imu_msg_ptr->linear_acceleration.y * 9.81,
                                                     imu_msg_ptr->linear_acceleration.z * 9.81);

        imu->gyro_ = Eigen::Vector3d(imu_msg_ptr->angular_velocity.x * M_PI / 180.0,
                                                        imu_msg_ptr->angular_velocity.y * M_PI / 180.0,
                                                        imu_msg_ptr->angular_velocity.z * M_PI / 180.0);
        imu->orientation.x = imu_msg_ptr->orientation.x;
        imu->orientation.y = imu_msg_ptr->orientation.y;
        imu->orientation.z = imu_msg_ptr->orientation.z;
        imu->orientation.w = imu_msg_ptr->orientation.w;
        double timestamp = imu->timestamp_;
        if (timestamp < last_timestamp_imu_) {
            LOG(WARNING) << "imu loop back, clear buffer";
            imu_buffer_.clear();
        }
        
        last_timestamp_imu_ = timestamp;

        imu_buffer_.emplace_back(imu);
    }

    /**
     * 处理观测uwb数据
     * @param 
     */
    void ProcessUwb(const cf_msgs::Tdoa::ConstPtr& msg) {
        UWBPtr uwb = std::make_shared<UWB>();
        uwb->timestamp_ = msg->header.stamp.toSec();
        uwb->idA_ = msg->idA;
        uwb->idB_ = msg->idB;
        uwb->data_ = msg->data;
        if (uwb->timestamp_ < last_timestamp_uwb_) {
            LOG(ERROR) << "lidar loop back, clear buffer";
            uwb_buffer_.clear();
        }
        
        uwb_buffer_.push_back(uwb);
        time_buffer_.push_back(uwb->timestamp_);
        last_timestamp_uwb_ = uwb->timestamp_;
        Sync();

    }
    std::mutex mtx_imu;
    std::mutex mtx_uwb;
    std::mutex mtx_sync;


   private:
    /// 尝试同步预测数据imu与观测数据uwb，成功时返回true
    bool Sync();

    Callback callback_;                             // 同步数据后的回调函数
    std::deque<UWBPtr> uwb_buffer_;                 // 观测uwb数据缓冲
    std::deque<IMUPtr> imu_buffer_;                 // imu数据缓冲
    double last_timestamp_imu_ = -1.0;              // 最近imu时间
    double last_timestamp_uwb_ = 0;                 // 最近uwb时间
    std::deque<double> time_buffer_;
    bool uwb_pushed_ = false;
    MeasureGroup measures_;                         // 同步后的数据
    double uwb_end_time_ = 0;
};

}  // namespace sad

#endif  //MEASURE_SYNC_H
