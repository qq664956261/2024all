#pragma once
#include <eigen3/Eigen/Dense>

struct ENcoderData {
  uint64_t exposure_ts;      //!< 编码器时间戳，单位us
  double v_l;                //!< 编码器左速度，单位m/s
  double v_r;               //!< 编码器右速度，单位m/s
};

struct ULtraData {
  uint64_t timestamp;         //!< 超声波时间戳，单位us
  uint32_t front_distance_l;  //!< 前方三合一左侧超声波距离，单位mm
  uint32_t front_distance_m;  //!< 前方三合一中间超声波距离，单位mm
  uint32_t front_distance_r;  //!< 前方三合一右侧超声波距离，单位mm
  uint8_t status;             //!< 超声波id
};

struct DEPData {
  uint64_t timestamp;         //!< 深度计时间戳，单位us
  int32_t pressure;          //!< 深度计压力值，单位pa
  int32_t temperature;       //!< 深度计温度值，单位。C
};

struct NAVISlamData {
  uint64_t timestamp;         //!< 时间戳，单位us
  uint8_t mode;              //!< 机器工作模式，0代表池底， 1代表水面
  uint8_t rotation_state;    //!< 旋转状态 0 means其他，1 means 旋转开始， 2 means 旋转结束
};

struct LEFTFrontData {
  uint64_t timestamp;         //!< 左前超声波时间戳，单位us
  uint32_t distance;          //!< 超声波距离，单位mm
  uint8_t status;           //!< 超声波状态
};

struct LEFTBackData {
  uint64_t timestamp;         //!< 左后超声波时间戳，单位us
  uint32_t distance;          //!< 超声波距离，单位mm
  uint8_t status;           //!< 超声波状态
};

struct LEFTTofData {
  uint64_t timestamp;         //!< 左侧tof时间戳，单位us
  uint32_t front_distance;    //!< 左前tof探测距离，单位mm
  uint32_t back_distance;    //!< 左后tof探测距离，单位mm
};

struct MAGData {
  uint64_t timestamp;        //!< 磁力计时间戳，单位us
  int16_t mag_x;            //!< 磁力计X轴强度
  int16_t mag_y;            //!< 磁力计Y轴强度
  int16_t mag_z;           //!< 磁力计Z轴强度
};

struct IMUData {
  uint64_t timestamp;
  float roll;
  float pitch;
  float yaw;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;

};

struct SOCIMUData {
  uint64_t timestamp;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;

};

/*!
 *  @brief 初始化算法库
 *  @param calibration_directory 一些标定文件存放的路径
 *  @param output_directory 是算法具有读写权限的路径，用于存储热启动文件
 *  @param vocabulary 词典路径，可用来做重定位
 */
int SensorFusionInitialImpl(const std::string calibration_directory, const std::string output_directory,
                            const std::string relocalization_directory, const std::string vocabulary);

int PutEncoderData(const ENcoderData &data);

int PutRelocalizationData(const Eigen::Matrix4d &relocalization_pose, const Eigen::Matrix4d &water_entry_pose, 
                          uint8_t relocalization_result, uint8_t build_frames_result, const uint8_t &curr_task);

int PutTaskData(const uint8_t &curr_task);

int PutImuData(const IMUData &imu_data);

int PutSocImuData(const SOCIMUData &soc_imu_data);

int PutMagData(const MAGData &mag_data);

int PutUltraData(const ULtraData &ultra_data);

int PutDepthData(const DEPData &data);

int PutLeftFrontUltraData(const LEFTFrontData &data);

int PutLeftBackUltraData(const LEFTBackData &data);

int PutLeftTofData(const LEFTTofData &data);

int PutNaviSlamData(const NAVISlamData &data);