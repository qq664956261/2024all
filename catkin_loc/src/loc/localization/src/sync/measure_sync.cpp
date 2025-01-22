//
// Created by zhang cheng on 2024/4/17.
//
#include "localization/sync/measure_sync.h"

namespace sad
{

    bool MessageSync::Sync()
    {
        // if (uwb_buffer_.empty() || imu_buffer_.empty()) {
        //     return false;
        // }
        if (uwb_buffer_.size() < 8 || imu_buffer_.empty())
        {
            return false;
        }
        UWB uwb0 = *uwb_buffer_.front();
        if(uwb0.idA_ !=0 )
        {
            uwb_buffer_.pop_front();
            time_buffer_.pop_front();
            return false;
        }
        measures_.uwb_.clear();
        if (!uwb_pushed_)
        {
            measures_.observation_uwb_ = *uwb_buffer_.front();
            measures_.observation_time_ = time_buffer_.front();
            uwb_end_time_ = measures_.observation_time_;
            uwb_pushed_ = true;
            while (measures_.uwb_.size() < 8 && !uwb_buffer_.empty())
            {
                measures_.uwb_.push_back(uwb_buffer_.front());
                uwb_buffer_.pop_front();
                time_buffer_.pop_front();
            }
        }

        if (last_timestamp_imu_ < uwb_end_time_)
        {
            return false;
        }

        double imu_time = imu_buffer_.front()->timestamp_;
        measures_.imu_.clear();
        while ((!imu_buffer_.empty()) && (imu_time < uwb_end_time_))
        {
            imu_time = imu_buffer_.front()->timestamp_;
            if (imu_time > uwb_end_time_)
            {
                break;
            }
            measures_.imu_.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        // uwb_buffer_.pop_front();
        // time_buffer_.pop_front();
        uwb_pushed_ = false;

        if (callback_)
        {
            callback_(measures_);
        }

        return true;
    }

} // namespace sad