#include "../../Fusion/Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#define SAMPLE_RATE (100) // replace this with actual sample rate
void readData(std::vector<FusionVector>& vacc, std::vector<FusionVector>& vgyr, std::vector<FusionVector>& veuler, std::vector<FusionVector>& vmag)
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

            // macc temp_acc;
            // temp_acc.ax = ax / 1000.0 * 9.81;
            // temp_acc.ay = ay / 1000.0 * 9.81;
            // temp_acc.az = az / 1000.0 * 9.81;
            FusionVector accelerometer = {ax / 1000.0 * 9.81, ay / 1000.0 * 9.81, az / 1000.0 * 9.81};
            vacc.push_back(accelerometer);
            // mgyr temp_gyr;
            // temp_gyr.wx = wx / 100.0 * 3.14 / 180.0;
            // temp_gyr.wy = wy / 100.0 * 3.14 / 180.0;
            // temp_gyr.wz = wz / 100.0 * 3.14 / 180.0;
            FusionVector gyroscope = {wx / 100.0 * 3.14 / 180.0, wy / 100.0 * 3.14 / 180.0, wz / 100.0 * 3.14 / 180.0};
            vgyr.push_back(gyroscope);
            // meuler temp_euler;
            // temp_euler.pitch = pitch / 100.0 * 3.14 / 180.0;
            // temp_euler.roll = roll / 100.0 * 3.14 / 180.0;
            // temp_euler.yaw = yaw / 100.0 * 3.14 / 180.0;
            FusionVector FusionEuler = {pitch / 100.0 * 3.14 / 180.0, roll / 100.0 * 3.14 / 180.0, yaw / 100.0 * 3.14 / 180.0};


            veuler.push_back(FusionEuler);


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

            // mmag temp_mag;
            // temp_mag.x = x;
            // temp_mag.y = y;
            // temp_mag.z = z;
            FusionVector magnetometer = {x, y, z}; 
            vmag.push_back(magnetometer);
           

        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    file_mag.close();

}

int main() {

    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    std::vector<FusionVector> vacc;
    std::vector<FusionVector> vgyr;
    std::vector<FusionVector> veuler;
    std::vector<FusionVector> vmag;

    readData(vacc, vgyr, veuler, vmag);

    // This loop should repeat each time new gyroscope data is available
    // while (true) {
        for(int i = 0; i < vacc.size(); i++)
        {

        // Acquire latest sensor data
        const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
        FusionVector gyroscope = vgyr[i]; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = vacc[i]; // replace this with actual accelerometer data in g
        FusionVector magnetometer = vmag[i]; // replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

        // Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static clock_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
               earth.axis.x, earth.axis.y, earth.axis.z);
    }
}
