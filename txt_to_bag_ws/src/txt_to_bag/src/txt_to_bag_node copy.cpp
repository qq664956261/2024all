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
#include "hj_interface/Pose.h"
#include "hj_interface/Nav.h"
#include "hj_interface/Mag.h"
#include "hj_interface/Imu.h"
#include "hj_interface/Encoder.h"
#include "hj_interface/Ultra.h"
#include "hj_interface/Depth.h"
#include "hj_interface/SocImu.h"
#include <tf/tf.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "hj_interface_converter");
    ros::NodeHandle nh;
    std::string path = argv[1];

    // 读取txt文件内容
    std::ifstream enc_file(path + "/encoder.log");
    std::ifstream imu_file(path + "/mcu_imu.log");
    std::ifstream uls_file(path + "/ultra.log");
    std::ifstream mag_file(path + "/mag.log");
    std::ifstream pressure_file(path + "/pressure.log");
    std::ifstream soc_imu_file(path + "/soc_imu.log");
    
    if (!imu_file.is_open()){
        ROS_ERROR("cannot open imu file !!!");
        return 1;
    }

    if(!enc_file.is_open()){
        ROS_ERROR("cannot open enc file !!!");
    }

    if(!uls_file.is_open()){
        ROS_ERROR("cannot open uls file !!!");
    }

    if(!mag_file.is_open()){
        ROS_ERROR("cannot open mag file !!!");
    }

    if(!pressure_file.is_open()){
        ROS_ERROR("cannot open pressure file !!!");
    }

    if(!soc_imu_file.is_open()){
        ROS_ERROR("cannot open soc_imu file !!!");
    }

    rosbag::Bag bag;
    bag.open(path + "/output.bag", rosbag::bagmode::Write);

    std::string imu_line, enc_line, uls_line, mag_line, pressure_line, soc_imu_line;
    // std::string imu_line, enc_line, gt_line;

    while(std::getline(imu_file, imu_line)){
        double imu_timestamp, index, roll, pitch, yaw, wx, wy, wz, ax, ay, az;
        std::istringstream imu_iss(imu_line);
        imu_iss >> imu_timestamp >> index >> roll >>pitch >>yaw >> wx >> wy >> wz >> ax >> ay >> az;
        // imu_iss >> imu_timestamp >> roll >>pitch >>yaw >> wx >> wy >> wz >> ax >> ay >> az;
        // 创建IMU消息
       hj_interface::Imu imu_msg;
       imu_msg.custom_time = ros::Time(imu_timestamp);
        if (imu_msg.custom_time.toSec() > 1.0){
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

  while(std::getline(soc_imu_file, soc_imu_line)) {
    double imu_timestamp, wx, wy, wz, ax, ay, az;
    std::istringstream soc_imu_iss(soc_imu_line);
    soc_imu_iss >> imu_timestamp >> ax >> ay >> az >> wx >> wy >> wz;
    // 创建IMU消息
    hj_interface::SocImu soc_imu_msg;
    soc_imu_msg.timestamp = ros::Time(imu_timestamp);

    if (soc_imu_msg.timestamp.toSec() > 1.0) {

      soc_imu_msg.gyro_x = wx;
      soc_imu_msg.gyro_y = wy;
      soc_imu_msg.gyro_z = wz;
      soc_imu_msg.accel_x = ax;
      soc_imu_msg.accel_y = ay;
      soc_imu_msg.accel_z = az;

      bag.write("/soc_imu_chatter", soc_imu_msg.timestamp , soc_imu_msg);
    }
  }

    int num = 0;
    while(std::getline(enc_file, enc_line)){
        double enc_timestamp, index, left_enc_data_d, right_enc_data_d;
        std::istringstream enc_iss(enc_line);
        enc_iss >> enc_timestamp >> index >> left_enc_data_d >> right_enc_data_d;
        // enc_iss >> enc_timestamp >> left_enc_data_d >> right_enc_data_d;

        hj_interface::Encoder enc_msg;
        enc_msg.custom_time = ros::Time(enc_timestamp);
        enc_msg.left_msg = left_enc_data_d;
        enc_msg.right_msg = right_enc_data_d;

        bag.write("/motor_chatter", enc_msg.custom_time, enc_msg);
    }

    while(std::getline(mag_file, mag_line)){
        double mag_timestamp, mag_x, mag_y, mag_z;
        std::istringstream mag_iss(mag_line);
        mag_iss >> mag_timestamp >> mag_x >> mag_y >> mag_z;

        // 创建码盘消息
        hj_interface::Mag mag_msg;
        mag_msg.custom_time = ros::Time(mag_timestamp);
        mag_msg.mag_x = mag_x;
        mag_msg.mag_y = mag_y;
        mag_msg.mag_z = mag_z;

        bag.write("/mag_chatter", mag_msg.custom_time, mag_msg);
    }

    
    while(std::getline(uls_file, uls_line)){
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
    while(std::getline(pressure_file, pressure_line)) {
        double depth_timestamp;
        double pressure, temperature;
        std::istringstream pressure_iss(pressure_line);
        pressure_iss >> depth_timestamp >> pressure >> temperature;
        hj_interface::Depth depth_msg;
        depth_msg.timestamp = ros::Time(depth_timestamp);
        depth_msg.pressure = static_cast<int32_t>(pressure);
        depth_msg.temp = static_cast<int32_t>(temperature);
        bag.write("/depth_chatter", depth_msg.timestamp, depth_msg);
    }
    bag.close();
    imu_file.close();
    enc_file.close();
    uls_file.close();
    mag_file.close();
    pressure_file.close();
    return 0;
}
