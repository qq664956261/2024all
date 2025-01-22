//
// Created by zhang cheng on 2024/4/17.
//
#include "localization/sync/measure_sync.h"

namespace sad
{

    bool MessageSync::Sync()
    {
        if (odom_buffer_.empty() || imu_buffer_.empty()) {
            return false;
        }


        if (!odom_pushed_)
        {
            measures_.observation_odom_ = *(odom_buffer_.front());
            measures_.observation_time_ = time_buffer_.front();
            odom_end_time_ = measures_.observation_time_;
            odom_pushed_ = true;

        }

        if (last_timestamp_imu_ < odom_end_time_)
        {
            return false;
        }

        double imu_time = imu_buffer_.front()->timestamp_;

        measures_.imu_.clear();
        while ((!imu_buffer_.empty()) && (imu_time < odom_end_time_))
        {
            imu_time = imu_buffer_.front()->timestamp_;
            if (imu_time > odom_end_time_)
            {
                break;
            }
            measures_.imu_.push_back(imu_buffer_.front());
            measures_.yaw_ = mag_yaw_.front();
            imu_buffer_.pop_front();
            mag_yaw_.pop_front();
        }

        odom_buffer_.pop_front();
        time_buffer_.pop_front();
        odom_pushed_ = false;

        if (callback_)
        {
            callback_(measures_);
        }

        return true;
    }

} // namespace sad