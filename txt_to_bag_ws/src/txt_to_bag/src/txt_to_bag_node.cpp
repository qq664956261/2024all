#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32MultiArray.h>
#include <typeinfo>
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Depth.h"
#include "hj_interface/SocImu.h"

#define X9_new

#include "hj_interface/Ultra.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/LeftFront.h"
#include "hj_interface/LeftBack.h"
#include "hj_interface/TripleUltra.h"
#include "hj_interface/LeftTof.h"

#include <tf/tf.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "txt_to_bag_converter");
  ros::NodeHandle nh;

  std::string path = argv[1];

  // 读取txt文件内容
  std::ifstream enc_file(path + "/encoder.log");
  std::ifstream imu_file(path + "/imu.log");

  std::ifstream mag_file(path + "/mag.log");
  std::ifstream depth_file(path + "/pressure.log");
  std::ifstream soc_imu_file(path + "/soc_imu.log");

#ifdef X9_old
  std::ifstream uls_file(path + "/ultra.log");
#elif defined(X9_new)
  std::ifstream triple_ultra_file(path + "/triple_ultra.log");
  std::ifstream left_front_file(path + "/left_front.log");
  std::ifstream left_back_file(path + "/left_back.log");

#elif defined(T1_pro)
  std::ifstream triple_ultra_file(path + "/triple_ultra.log");
  std::ifstream left_tof_file(path + "/left_tof.log");

#endif
  if(!enc_file.is_open()) {
    ROS_ERROR("cannot open enc file !!!");
    return -1;
  }

  if (!imu_file.is_open()) {
    ROS_ERROR("cannot open imu file !!!");
    return -1;
  }
  // if(!soc_imu_file.is_open()) {
  //   ROS_ERROR("cannot open soc imu file !!!");
  //   return -1;
  // }

  if(!mag_file.is_open()) {
    ROS_ERROR("cannot open mag file !!!");
    return -1;
  }

  if(!depth_file.is_open()) {
    ROS_ERROR("cannot open depth file !!!");
    return -1;
  }

#ifdef X9_old
  if(!uls_file.is_open()) {
      ROS_ERROR("cannot open uls file !!!");
  }
#elif defined(X9_new)
  if(!triple_ultra_file.is_open()) {
    ROS_ERROR("cannot open triple ultra file !!!");
  }
  if(!left_front_file.is_open()) {
      ROS_ERROR("cannot open left front uls file !!!");
  }
  if(!left_back_file.is_open()) {
    ROS_ERROR("cannot open left back uls file !!!");
  }
#elif defined(T1_pro)
  if(!triple_ultra_file.is_open()) {
    ROS_ERROR("cannot open triple ultra file !!!");
  }
  if(!left_tof_file.is_open()) {
      ROS_ERROR("cannot open left tof file !!!");
  }

#endif

  rosbag::Bag bag;
  bag.open(path + "/output.bag", rosbag::bagmode::Write);

  std::string imu_line, enc_line, uls_line, mag_line, depth_line,
  soc_imu_line, left_front_line, left_back_line, triple_ultra_line, left_tof_line;

  //write encoder data
  while(std::getline(enc_file, enc_line)) {
    double enc_timestamp,index, left_enc_data_d, right_enc_data_d;
    std::istringstream enc_iss(enc_line);
    enc_iss >> enc_timestamp >> index>> left_enc_data_d >> right_enc_data_d;
    //enc_iss >> enc_timestamp >>left_enc_data_d >> right_enc_data_d;
    hj_interface::Encoder enc_msg;
    enc_msg.custom_time = ros::Time(enc_timestamp);
    enc_msg.left_msg = left_enc_data_d;
    enc_msg.right_msg = right_enc_data_d;
    bag.write("/motor_chatter", enc_msg.custom_time, enc_msg);
  }

  //write imu data
  while(std::getline(imu_file, imu_line)) {
    double imu_timestamp, index, roll, pitch, yaw, wx, wy, wz, ax, ay, az;
    std::istringstream imu_iss(imu_line);
    imu_iss >> imu_timestamp >> index >>roll >>pitch >>yaw >> wx >> wy >> wz >> ax >> ay >> az;
    //imu_iss >> imu_timestamp >> roll >>pitch >>yaw >> wx >> wy >> wz >> ax >> ay >> az;
    // 创建IMU消息
    hj_interface::Imu imu_msg;
    imu_msg.custom_time = ros::Time(imu_timestamp);
    if (imu_msg.custom_time.toSec() > 1.0) {
      imu_msg.roll = roll;
      imu_msg.pitch = pitch;
      imu_msg.yaw = yaw;
      imu_msg.accel_x = ax;
      imu_msg.accel_y = ay;
      imu_msg.accel_z = az;
      imu_msg.gyro_x = wx;
      imu_msg.gyro_y = wy;
      imu_msg.gyro_z = wz;
      bag.write("/imu_chatter", imu_msg.custom_time , imu_msg);
    }
  }

//write soc imu data
  // while(std::getline(soc_imu_file, soc_imu_line)) {
  //   double imu_timestamp, wx, wy, wz, ax, ay, az;
  //   std::istringstream soc_imu_iss(soc_imu_line);
  //   soc_imu_iss >> imu_timestamp >> ax >> ay >> az >> wx >> wy >> wz;
  //   // 创建IMU消息
  //   hj_interface::SocImu soc_imu_msg;
  //   soc_imu_msg.timestamp = ros::Time(imu_timestamp);
  //   if (soc_imu_msg.timestamp.toSec() > 1.0) {
  //     soc_imu_msg.gyro_x = wx;
  //     soc_imu_msg.gyro_y = wy;
  //     soc_imu_msg.gyro_z = wz;
  //     soc_imu_msg.accel_x = ax;
  //     soc_imu_msg.accel_y = ay;
  //     soc_imu_msg.accel_z = az;
  //     bag.write("/soc_imu_chatter", soc_imu_msg.timestamp , soc_imu_msg);
  //   }
  // }

  //write mag data
  while(std::getline(mag_file, mag_line)) {
    double mag_timestamp, mag_x, mag_y, mag_z;
    std::istringstream mag_iss(mag_line);
    mag_iss >> mag_timestamp >> mag_x >> mag_y >> mag_z;
    hj_interface::Mag mag_msg;
    mag_msg.custom_time = ros::Time(mag_timestamp);
    mag_msg.mag_x = mag_x;
    mag_msg.mag_y = mag_y;
    mag_msg.mag_z = mag_z;
    bag.write("/mag_chatter", mag_msg.custom_time, mag_msg);
  }

  //write depth data
  while(std::getline(depth_file, depth_line)) {
    double depth_timestamp;
    double pressure, temperature;
    std::istringstream pressure_iss(depth_line);
    pressure_iss >> depth_timestamp >> pressure >> temperature;
    hj_interface::Depth depth_msg;
    depth_msg.timestamp = ros::Time(depth_timestamp);
    depth_msg.pressure = static_cast<int32_t>(pressure);
    depth_msg.temp = static_cast<int32_t>(temperature);
    bag.write("/depth_chatter", depth_msg.timestamp, depth_msg);
  }

#ifdef X9_old
  while(std::getline(uls_file, uls_line)) {
    double uls_timestamp;
    double f_range_l, f_range_m, f_range_r,m_range,b_range,id;
    std::istringstream uls_iss(uls_line);
    uls_iss >> uls_timestamp >> f_range_l >> f_range_m >> f_range_r>>m_range >> b_range>>id;
    hj_interface::Ultra ultra_msg;
    ultra_msg.timestamp = ros::Time(uls_timestamp);
    ultra_msg.front_l = f_range_l;
    ultra_msg.front_m = f_range_m;
    ultra_msg.front_r = f_range_r;
    ultra_msg.mid = m_range;
    ultra_msg.back = b_range;
    ultra_msg.id = id;
    bag.write("/ulsound_chatter", ultra_msg.timestamp, ultra_msg);
  }
#elif defined(X9_new)
  while(std::getline(triple_ultra_file, triple_ultra_line)) {
    double triple_ultra_timestamp;
    double triple_left_ultra, triple_middle_ultra, triple_right_ultra, status;
    std::istringstream triple_ultra_iss(triple_ultra_line);
    triple_ultra_iss >> triple_ultra_timestamp >> triple_left_ultra >> triple_middle_ultra >> triple_right_ultra >>status;
    hj_interface::TripleUltra triple_ultra_msg;
    triple_ultra_msg.timestamp = ros::Time(triple_ultra_timestamp);
    triple_ultra_msg.front_l = static_cast<uint32_t>(triple_left_ultra);
    triple_ultra_msg.front_m = static_cast<uint32_t>(triple_middle_ultra);
    triple_ultra_msg.front_r = static_cast<uint32_t>(triple_right_ultra);
    triple_ultra_msg.status = static_cast<uint8_t>(status);
    bag.write("/triple_ultra", triple_ultra_msg.timestamp, triple_ultra_msg);
  }

  while(std::getline(left_front_file, left_front_line)) {
    double left_front_timestamp;
    double left_front_distance, status;
    std::istringstream left_front_iss(left_front_line);
    left_front_iss >> left_front_timestamp >> left_front_distance >>status;
    hj_interface::LeftFront left_front_msg;
    left_front_msg.timestamp = ros::Time(left_front_timestamp);
    left_front_msg.dist = static_cast<uint32_t>(left_front_distance);
    left_front_msg.status = static_cast<uint8_t>(status);
    bag.write("/x9/left_front", left_front_msg.timestamp, left_front_msg);
  }

  while(std::getline(left_back_file, left_back_line)) {
    double left_back_timestamp;
    double left_back_distance, status;
    std::istringstream left_back_iss(left_back_line);
    left_back_iss >> left_back_timestamp >> left_back_distance >>status;
    hj_interface::LeftBack left_back_msg;
    left_back_msg.timestamp = ros::Time(left_back_timestamp);
    left_back_msg.dist = static_cast<uint32_t>(left_back_distance);
    left_back_msg.status = static_cast<uint8_t>(status);
    bag.write("/x9/left_back", left_back_msg.timestamp, left_back_msg);
  }


#elif defined(T1_pro)

  while(std::getline(triple_ultra_file, triple_ultra_line)) {
    double triple_ultra_timestamp;
    double triple_left_ultra, triple_middle_ultra, triple_right_ultra, status;
    std::istringstream triple_ultra_iss(triple_ultra_line);
    triple_ultra_iss >> triple_ultra_timestamp >> triple_left_ultra >> triple_middle_ultra >> triple_right_ultra >>status;
    hj_interface::TripleUltra triple_ultra_msg;
    triple_ultra_msg.timestamp = ros::Time(triple_ultra_timestamp);
    triple_ultra_msg.front_l = static_cast<uint32_t>(triple_left_ultra);
    triple_ultra_msg.front_m = static_cast<uint32_t>(triple_middle_ultra);
    triple_ultra_msg.front_r = static_cast<uint32_t>(triple_right_ultra);
    triple_ultra_msg.status = static_cast<uint8_t>(status);
    bag.write("/triple_ultra", triple_ultra_msg.timestamp, triple_ultra_msg);
  }

  while(std::getline(left_tof_file, left_tof_line)) {
    double left_tof_timestamp;
    double left_tof_front_distance, left_tof_back_distance;
    std::istringstream left_tof_iss(left_tof_line);
    left_tof_iss >> left_tof_timestamp >> left_tof_front_distance >>left_tof_back_distance;
    hj_interface::LeftTof left_tof_msg;
    left_tof_msg.timestamp = ros::Time(left_tof_timestamp);
    left_tof_msg.dist_front = static_cast<uint32_t>(left_tof_front_distance);
    left_tof_msg.dist_back = static_cast<uint32_t>(left_tof_back_distance);
    bag.write("/t1pro/left_tof", left_tof_msg.timestamp, left_tof_msg);
  }


#endif

  bag.close();
  imu_file.close();
  enc_file.close();
#ifdef X9_old
  uls_file.close();
#elif defined(X9_new)
  triple_ultra_file.close();
  left_front_file.close();
  left_back_file.close();
#elif defined(T1_pro)
  triple_ultra_file.close();
  left_tof_file.close();
#endif
  mag_file.close();
  depth_file.close();

  return 0;
}
