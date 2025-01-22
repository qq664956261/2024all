// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT

// This is a simple main function that runs VQF with simple dummy data and a few different options.
// The main purpose of this file is to test compilation and to have an executable to run memtest:
//     valgrind --tool=memcheck --track-origins=yes --leak-check=full ./vqf_dummy


#include "vqf.hpp"
#include "basicvqf.hpp"
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
#include "offline_vqf.hpp"
#endif
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
struct macc
{
    double ax;
    double ay;
    double az;
};
struct mgyr
{
    double wx;
    double wy;
    double wz;
};

struct meuler
{
    double roll;
    double pitch;
    double yaw;
};
struct mmag
{
    double x;
    double y;
    double z;
};

VQFParams getParams(int mode)
{
    VQFParams params;

    switch (mode) {
    case 0:
        break; // default params
    case 1:
        params.magDistRejectionEnabled = false;
        break;
    case 2:
        params.restBiasEstEnabled = false;
        break;
    case 3:
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        params.motionBiasEstEnabled = false;
#endif
        break;
    case 4:
        params.restBiasEstEnabled = false;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        params.motionBiasEstEnabled = false;
#endif
        break;
    case 5:
        params.magDistRejectionEnabled = false;
        params.restBiasEstEnabled = false;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        params.motionBiasEstEnabled = false;
#endif
        break;
    default:
        std::cout << "invalid mode!" << std::endl;
    }

    return params;
}

void run(int mode)
{
    vqf_real_t gyr[3] = {0.01, 0.01, 0.01};
    vqf_real_t acc[3] = {0, 9.8, 0};
    vqf_real_t mag[3] = {0.5, 0.8, 0};
    vqf_real_t quat[4] = {0, 0, 0, 0};

    VQFParams params = getParams(mode);

    VQF vqf(params, 0.01);
    for (size_t i=0; i < 10000; i++) {
        vqf.update(gyr, acc, mag);
        vqf.getQuat9D(quat);
    }

    // std::cout << "VQF, mode: " << mode << ", quat: [" <<
    //              quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << std::endl;
}
void myrun(int mode, std::vector<macc>& vacc, std::vector<mgyr>& vgyr, std::vector<meuler>& veuler, std::vector<mmag>& vmag)
{
    double bias_z = 0;
    VQFParams params = getParams(mode);
    bool first_imu = true;
    Eigen::Quaterniond q21;
    VQF vqf(params, 0.01);
    //vqf_real_t hard_iron_bias[3] = {73.83746385, 64.73454899, 76.04280512};
    // vqf_real_t hard_iron_bias[3] = {323.82, 29.42, 76.04280512};
    // vqf_real_t hard_iron_bias[3] = {188.31, 493.11, 28.30};
    //  vqf_real_t hard_iron_bias[3] = {217.46, 523.92, 28.30};
    //vqf_real_t hard_iron_bias[3] = {178.77, 441.36, 28.30};
    vqf_real_t hard_iron_bias[3] = {267.9, 78.82, 75.12};
    vqf_real_t soft_iron[9] = {4.57823669, 1.27409684e-02, 3.67589785e-03, 1.27409684e-02, 4.96063023e+00, 2.38515112e-02, 3.67589785e-03, 2.38515112e-02, 6.74932603e+00};
    std::ofstream out("./1.txt",std::ios::app);
    std::ofstream out1("./2.txt",std::ios::app);
    std::ofstream out2("./bias.txt",std::ios::app);
    std::ofstream out4("./4.txt",std::ios::app);
    for (int i = 0; i < vacc.size(); i++) {
        //std::cout<<"az:"<<vacc[i].az<<std::endl;
  
        vqf_real_t quat[4] = {0, 0, 0, 0};
        vqf_real_t bias[3] = {0 ,0, 0};
        vqf_real_t gyr[3] = {vgyr[i].wx, vgyr[i].wy, vgyr[i].wz};
        vqf_real_t acc[3] = {vacc[i].ax, vacc[i].ay, vacc[i].az};
        //vqf_real_t mag[3] = {vmag[i].x, vmag[i].y, vmag[i].z};

        vqf_real_t mag[3] = {(vmag[i].x - hard_iron_bias[0]) , (vmag[i].y - hard_iron_bias[1]) , (vmag[i].z - hard_iron_bias[2])};
        if(std::fabs(veuler[i].roll) > 0.17 || std::fabs(veuler[i].pitch) > 0.17)
        {
            std::cout<<"veuler[i].roll:"<<veuler[i].roll<<std::endl;
            std::cout<<"veuler[i].pitch:"<<veuler[i].pitch<<std::endl;

            mag[0] = 0.0;
            mag[1] = 0.0;
            mag[2] = 0.0;


        }
        else
        {
            std::ofstream out3("./3.txt",std::ios::app);
            out3<<atan2(mag[1], mag[0])<<std::endl;
        }
        vqf.getQuat9D(quat);
        std::cout << "VQF, mode: " << mode << ", quat: [" <<
                 quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << std::endl;
        vqf.update(gyr, acc, mag);
        
        vqf.getBiasEstimate(bias);

            // 计算 Yaw (Z轴旋转)
    double siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
    double cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

        // 计算 Roll (X轴旋转)
    double sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
    double cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    
    // 计算 Pitch (Y轴旋转)
    double pitch;
    double sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // 防止超出范围，使用 ±90 度
    else
        pitch = std::asin(sinp);

    Eigen::AngleAxisd rotation_vector(1.57, Eigen::Vector3d(0, 0, 1)); 
    Eigen::Matrix3d R = rotation_vector.matrix();
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(veuler[i].pitch,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(veuler[i].roll,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(veuler[i].yaw,Eigen::Vector3d::UnitZ())); 
    Eigen::Matrix3d rotation_matrix=yawAngle.toRotationMatrix()*pitchAngle.toRotationMatrix()*rollAngle.toRotationMatrix();
    Eigen::Quaterniond q1(R * rotation_matrix);
    Eigen::Quaterniond q2(quat[0],  quat[1],  quat[2],  quat[3]);
    if(first_imu && yaw != 0)
    {
        first_imu = false;
        q21 = q2 * q1.inverse();

    }
    q1 = q21 * q1;
            std::cout << "imu, mode: " << mode << ", quat: [" <<
                 q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << "]" << std::endl;
    std::cout<<"q1 yaw:"<<q1.toRotationMatrix().eulerAngles(2,1,0)<<std::endl;
    std::cout<<"q2 yaw:"<<q2.toRotationMatrix().eulerAngles(2,1,0)<<std::endl;
    //std::cout<<"q2 yaw:"<<q2.toRotationMatrix()<<std::endl;
    std::cout<<"q1.inverse() * q2:"<<q1.toRotationMatrix().inverse() * q2.toRotationMatrix()<<std::endl;

    //std::cout<<"yaw:"<<yaw<<std::endl;
    double siny_cosp1 = 2 * (q1.w() * q1.z() + q1.x() * q1.y());
    double cosy_cosp1 = 1 - 2 * (q1.y() * q1.y() + q1.z() * q1.z());
    double yaw1 = std::atan2(siny_cosp1, cosy_cosp1);

    out<<yaw<<std::endl;
    out4<<pitch<<std::endl;
    out1 << yaw1<< std::endl;
    out2 << bias[2] << std::endl;



    }


}

void runBasic()
{
    vqf_real_t gyr[3] = {0.01, 0.01, 0.01};
    vqf_real_t acc[3] = {0, 9.8, 0};
    vqf_real_t mag[3] = {0.5, 0.8, 0};
    vqf_real_t quat[4] = {0, 0, 0, 0};

    BasicVQF vqf(0.01);
    for (size_t i=0; i < 10000; i++) {
        vqf.update(gyr, acc, mag);
        vqf.getQuat9D(quat);
    }

    std::cout << "BasicVQF, quat: [" <<
                 quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << std::endl;
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION // setting this disables functions needed by offline estimation
void runOffline(int mode)
{
    size_t N = 10000;
    vqf_real_t *gyr = new vqf_real_t[N*3]();
    vqf_real_t *acc = new vqf_real_t[N*3]();
    vqf_real_t *mag = new vqf_real_t[N*3]();
    vqf_real_t *quat = new vqf_real_t[N*4]();
    for (size_t i = 0; i < N; i++) {
        gyr[3*i] = 0.01;
        gyr[3*i + 1] = 0.01;
        gyr[3*i + 2] = 0.01;
        acc[3*i] = 0;
        acc[3*i + 1] = 9.8;
        acc[3*i + 2] = 0;
        mag[3*i] = 0.5;
        mag[3*i + 1] = 0.8;
        mag[3*i + 2] = 0;
    }

    VQFParams params = getParams(mode);

    offlineVQF(gyr, acc, mag, N, 0.01, params, 0, quat, 0, 0, 0, 0, 0);

    std::cout << "offlineVQF, mode: " << mode << ", quat[0]: [" <<
                 quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3] << "]" << ", quat[N]: [" <<
                 quat[4*(N-1)] << ", " << quat[4*(N-1)+1] << ", " << quat[4*(N-1)+2] << ", " << quat[4*(N-1)+3] <<
                 "]" << std::endl;
    delete[] acc;
    delete[] gyr;
    delete[] mag;
    delete[] quat;
}
#endif


void readData(std::vector<macc>& vacc, std::vector<mgyr>& vgyr, std::vector<meuler>& veuler, std::vector<mmag>& vmag)
{
    // std::ifstream file("/home/zc/data/rect/10261126/userdata/hj/log/sensor_data_alg/pool_bottom/imu.log");
    // std::ifstream file_mag("/home/zc/data/rect/10261126/userdata/hj/log/sensor_data_alg/pool_bottom/mag.log");
    // std::ifstream file("//home/zc/data/logs_ScubaX1ProMax_X9X44600048_20241210_164256/userdata/hj/log/sensor_data_alg/pool_wall/imu.log");
    // std::ifstream file_mag("/home/zc/data/logs_ScubaX1ProMax_X9X44600048_20241210_164256/userdata/hj/log/sensor_data_alg/pool_wall/mag.log");
    // std::ifstream file("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_025918/userdata/hj/log/sensor_data_alg/pool_bottom/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_025918/userdata/hj/log/sensor_data_alg/pool_bottom/mag.log");
    // std::ifstream file("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_032731/userdata/hj/log/sensor_data_alg/pool_wall/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_032731/userdata/hj/log/sensor_data_alg/pool_wall/mag.log");
    // std::ifstream file("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_043018/userdata/hj/log/sensor_data_alg/pool_wall/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_18/logs_ScubaX1ProMax_X9X45000054_20241218_043018/userdata/hj/log/sensor_data_alg/pool_wall/mag.log");
    // std::ifstream file("/home/zc/data/12_19/logs_ScubaX1ProMax_X9X45000054_20241219_025424/userdata/hj/log/sensor_data_alg/data/pool_bottom/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_19/logs_ScubaX1ProMax_X9X45000054_20241219_025424/userdata/hj/log/sensor_data_alg/data/pool_bottom/mag.log");
    // std::ifstream file("/home/zc/data/12_19/1750/userdata/hj/log/sensor_data_alg/data/pool_bottom/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_19/1750/userdata/hj/log/sensor_data_alg/data/pool_bottom/mag.log");
    std::ifstream file("/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241220_100903/userdata/hj/log/sensor_data_alg/data/pool_bottom/imu.log");
    std::ifstream file_mag("/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241220_100903/userdata/hj/log/sensor_data_alg/data/pool_bottom/mag.log");
    // std::ifstream file("/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241221_080153/userdata/hj/log/sensor_data_alg/data/pool_bottom/imu.log");
    // std::ifstream file_mag("/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241221_080153/userdata/hj/log/sensor_data_alg/data/pool_bottom/mag.log");
    
    if (!file.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return ;
    }

    std::string line;

    

    while (std::getline(file, line)) {
        std::istringstream iss(line);

        double imu_timestamp, index, roll, pitch, yaw, wx, wy, wz, ax, ay, az;
        // 读取每行的数据到 IMUData 结构
        if (iss >> imu_timestamp >> index
                >> roll >> pitch >> yaw
                >> wx >> wy >> wz
                >> ax >> ay >> az) {

            macc temp_acc;
            temp_acc.ax = ax / 1000.0 * 9.81;
            temp_acc.ay = ay / 1000.0 * 9.81;
            temp_acc.az = az / 1000.0 * 9.81;
            vacc.push_back(temp_acc);
            mgyr temp_gyr;
            temp_gyr.wx = wx / 100.0 * 3.14 / 180.0;
            temp_gyr.wy = wy / 100.0 * 3.14 / 180.0;
            temp_gyr.wz = wz / 100.0 * 3.14 / 180.0;
            vgyr.push_back(temp_gyr);
            meuler temp_euler;
            temp_euler.pitch = pitch / 100.0 * 3.14 / 180.0;
            temp_euler.roll = roll / 100.0 * 3.14 / 180.0;
            temp_euler.yaw = yaw / 100.0 * 3.14 / 180.0;


            veuler.push_back(temp_euler);


        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    file.close();

        while (std::getline(file_mag, line)) {
        std::istringstream iss(line);

        double mag_timestamp, x, y, z;
        // 读取每行的数据到 mag 结构
        if (iss >> mag_timestamp >> x
                >> y >> z) {

            mmag temp_mag;
            temp_mag.x = x;
            temp_mag.y = y;
            temp_mag.z = z;
            vmag.push_back(temp_mag);
           

        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    file_mag.close();

}

int main()
{
    for (int mode=0; mode <= 5; mode++) {
        //run(mode);
    }
    //runBasic();

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    for (int mode=0; mode <= 5; mode++) {
        //runOffline(mode);
    }
#endif
    std::vector<macc> vacc;
    std::vector<mgyr> vgyr;
    std::vector<meuler> veuler;
    std::vector<mmag> vmag;
    readData(vacc,vgyr,veuler,vmag);

    myrun(0, vacc, vgyr,veuler,vmag );


    return 0;
}
