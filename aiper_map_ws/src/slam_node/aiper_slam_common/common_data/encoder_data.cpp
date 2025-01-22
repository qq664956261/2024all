#include "encoder_data.h"
//#include "log.h"
namespace common_data {
EncoderData::EncoderData(const uint64_t time_, const double v_l_, const double v_r_)
        : time(time_), v_l(v_l_), v_r(v_r_) {
  acc = Eigen::Vector3d::Zero();
  gyro = Eigen::Vector3d::Zero();
  euler = Eigen::Vector3d::Zero();

}

uint64_t EncoderData::getTime() {
  return time;
}

double EncoderData::getVelocityLeft() {
  return v_l;
}

double EncoderData::getVelocityRight() {
  return v_r;
}

Eigen::Vector3d EncoderData::getAcc() {
  return acc;
}

Eigen::Vector3d EncoderData::getGyro() {
  return gyro;
}

Eigen::Vector3d EncoderData::getEuler() {
  return euler;
}

Eigen::Vector3d EncoderData::calDeltaEuler(EncoderDataPtr &last, EncoderDataPtr &curr) {
  Eigen::Vector3d last_euler = last->getEuler();
  Eigen::Vector3d curr_euler = curr->getEuler();
  Eigen::Vector3d delta_euler = Eigen::Vector3d::Zero();
  delta_euler = curr_euler - last_euler;
  bool flag = last_euler[2] * curr_euler[2] >= 0.0 ? 1 : 0;
  if (flag) {
    if (fabs(delta_euler[2]) > 10.0) {
      //HJ_ERROR("function: %s", __FUNCTION__);
      //HJ_ERROR("delta euler is too large, euler angle is %lf, %lf", last_euler[2], curr_euler[2]);
      //HJ_ERROR("delta timestamp diff is too large, timestamp is %ld, %ld", last->getTime(), curr->getTime());
    }
    return delta_euler;
  }
  else {
    if (delta_euler[2] > 300.0) {
      double sign = -1.0;
      delta_euler[2] = sign * (360.0 - delta_euler[2]);
    }
    if (delta_euler[2] < -300.0) {
      double sign = 1.0;
      delta_euler[2] = sign * (360.0 + delta_euler[2]);
    }
    if (fabs(delta_euler[2]) > 10.0) {
      //HJ_ERROR("function: %s", __FUNCTION__);
      //HJ_ERROR("delta euler is too large, euler angle is %lf, %lf", last_euler[2], curr_euler[2]);
      //HJ_ERROR("delta timestamp diff is too large, timestamp is %ld, %ld", last->getTime(), curr->getTime());
    }
    return delta_euler;
  }


}

Eigen::Vector3d EncoderData::calAveEuler(EncoderDataPtr &last, EncoderDataPtr &curr) {
  Eigen::Vector3d last_euler = last->getEuler();
  Eigen::Vector3d curr_euler = curr->getEuler();
  double last_yaw = last_euler[2] < 0.0 ? last_euler[2] + 360.0 : last_euler[2];
  double curr_yaw = curr_euler[2] < 0.0 ? curr_euler[2] + 360.0 : curr_euler[2];
  last_euler[2] = last_yaw;
  curr_euler[2] = curr_yaw;
  Eigen::Vector3d aveEuler = 0.5 * (last_euler + curr_euler);
  if (aveEuler[2] > 180.0)
    aveEuler[2] -= 360.0;
  return aveEuler;

}

bool EncoderData::interpolationAccGyroEuler(uint64_t ts, ImuMeasurements &imus) {
  bool find_imu_older = false;
  bool find_imu_newer = false;
  ImuData m_low;
  ImuData m_high;
  for (std::deque<ImuData, Eigen::aligned_allocator<ImuData>>::reverse_iterator rit = imus.rbegin(); rit!=imus.crend(); ++rit){
    if ((*rit).time >= ts) {
      m_high = (*rit);
      find_imu_newer = true;
      break;
    }
  }
  for (std::deque<ImuData, Eigen::aligned_allocator<ImuData>>::iterator rit = imus.begin(); rit != imus.end(); ++rit){
    if ((*rit).time < ts) {
      m_low = (*rit);
      find_imu_older = true;
      break;
    }
  }
  if (find_imu_newer == true && find_imu_older == true) {
    double k1 = (static_cast<double>(ts - m_low.time)) / (static_cast<double>(m_high.time - m_low.time));
    double k2 = (static_cast<double>(m_high.time - ts)) / (static_cast<double>(m_high.time - m_low.time));
    acc = k1 * m_high.linear_acceleration_ + k2 * m_low.linear_acceleration_;
    gyro = k1 * m_high.angular_velocity_ + k2 * m_low.angular_velocity_;
    euler = k1 * m_high.euler_angle_ + k2 * m_low.euler_angle_;
    double imu_yaw;
    if (m_low.euler_angle_[2] < -170 && m_high.euler_angle_[2] > 170) {
      double a = 180 - fabs(m_low.euler_angle_[2]);
      double b = 180 - fabs(m_high.euler_angle_[2]);
      double yaw = m_low.euler_angle_[2] - (k2 * a + k1 * b) / (k1 + k2);
      euler[2] = yaw;
      if (yaw < -180.0) {
        double diff = fabs(yaw) - 180.0;
        imu_yaw = 180.0 - diff;
        euler[2] = imu_yaw;
      }
    }
    if (m_low.euler_angle_[2] > 170 && m_high.euler_angle_[2] < -170) {
      double a = 180 - fabs(m_low.euler_angle_[2]);
      double b = 180 - fabs(m_high.euler_angle_[2]);
      double yaw = m_low.euler_angle_[2] + (k2 * a + k1 * b) / (k1 + k2);
      euler[2] = yaw;
      if (yaw > 180.0) {
        double diff = fabs(yaw) - 180.0;
        imu_yaw = -180.0 + diff;
        euler[2] = imu_yaw;
      }
    }
    if (m_low.euler_angle_[2] < 0 && m_low.euler_angle_[2] > -10 && m_high.euler_angle_[2] > 0 && m_high.euler_angle_[2] < 10) {
      double a = fabs(m_low.euler_angle_[2]);
      double b = fabs(m_high.euler_angle_[2]);
      double yaw = m_low.euler_angle_[2] + (k2 * a + k1 * b) / (k1 + k2);
      euler[2] = yaw;
    }
    if (m_low.euler_angle_[2] < 10 && m_low.euler_angle_[2] > 0 && m_high.euler_angle_[2] < 0 && m_high.euler_angle_[2] > -10) {
      double a = fabs(m_low.euler_angle_[2]);
      double b = fabs(m_high.euler_angle_[2]);
      double yaw = m_low.euler_angle_[2] - (k2 * a + k1 * b) / (k1 + k2);
      euler[2] = yaw;
    }
  return true;
  }
  else if (find_imu_newer == false && find_imu_older == true) {
    acc = m_low.linear_acceleration_;
    gyro =  m_low.angular_velocity_;
    euler = m_low.euler_angle_;
    return true;
  }
  else if (find_imu_newer == true && find_imu_older == false) {
    acc = m_high.linear_acceleration_;
    gyro =  m_high.angular_velocity_;
    euler = m_high.euler_angle_;
    return true;
  }
  else {
    //HJ_WARN("could not interpolation acc gyro euler in encoder time!");
    return false;
  }
  return true;

}


EncoderData::~EncoderData() {
  acc = Eigen::Vector3d::Zero();
  gyro = Eigen::Vector3d::Zero();
  euler = Eigen::Vector3d::Zero();
}
}
