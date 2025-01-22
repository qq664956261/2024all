#ifndef SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_
#define SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_
#include <unordered_map>
#include <string>

typedef enum {
  // collect_node error code
  SOC_IMU_INIT_ERROR = 1000,               // soc_Imu启动异常
  SOC_IMU_DATA_ERROR = 1001,               // soc_Imu数据读取连续空值
  MAG_INIT_ERROR = 1002,                   // 磁力计启动异常
  MAG_DATA_ERROR = 1003,                   // 磁力计数据读取连续空值
  TUR_INIT_ERROR = 1004,                   // 浊度计初始化失败
  TUR_DATA_ERROR = 1005,                   // 浊度计数据读取连续空值
  ULSOUND_FRONT_USS_INIT_ERROR = 1006,     // 三合一超声启动异常
  ULSOUND_FRONT_USS_DATA_ERROR = 1007,     // 三合一超声数据读取连续空值
  ULSOUND_SIDE_MID_INIT_ERROR = 1008,      // 侧边前超声启动异常
  ULSOUND_SIDE_MID_DATA_ERROR = 1009,      // 侧边前超声数据读取连续空值
  ULSOUND_SIDE_BACK_DATA_ERROR = 1010,     // 侧边后面超声数据读取连续空值
  TOF_INIT_ERROR = 1011,                   // tof启动异常
  SIDE_TOF_DATA_ERROR = 1012,              // 侧边tof读取数据失败
  UNDER_TOF_DATA_ERROR = 1013,             // 下方tof读取数据失败
  WF5803_INIT_ERROR = 1014,                // 压力计启动异常
  WF5803_DATA_ERROR = 1015,                // 压力计数据读取连续空值
  BT_ERROR = 1016,                         // 蓝牙连接异常
  ULSOUND_SIDE_BACK_INIT_ERROR = 1017,     // 侧边后面超声启动异常
  ULSOUND_DOWN_LEFT_DATA_ERROR = 1018,     // 下面超声数据读取连续空值
  ULSOUND_DOWN_INIT_ERROR = 1019,          // 下面超声启动异常
  TUR_DATA_OVERFLOW_ERROR = 1020,          // 浊度计数据持续满值
  COLLECT_NODE_COREDUMP_ERROR = 1994,       // collect节点coredump
  MIDDLEWARE_NODE_COREDUMP_ERROR = 1995,    // middleware节点coredump
  SLAM_NODE_COREDUMP_ERROR = 1996,          // slam节点coredump
  PLANING_NODE_COREDUMP_ERROR = 1997,       // 规划节点coredump
  UTILS_NODE_COREDUMP_ERROR = 1998,         // 工具节点coredump
  COLLECT_NODE_MAX = 1999,                 // collect node error code max


  // mcu/midleware_ndoe error code
  MCU_INIT_ERROR = 2000,
  LEFT_WHEEL_ERROR,       // 左驱动轮状态
  RIGHT_WHEEL_ERROR,      // 右驱动轮状态
  STEERING_MOTOR_ERROR,   // 转向电机状态
  LEFT_WATER_PUMP_ERROR,  // 左水泵状态
  RIGHT_WATER_PUMP_ERROR,  // 右水泵状态
  AIRBAG_ERROR,            // 气囊电机状态
  FAN_ERROR,               // 风机状态
  FLIP_COVER_MOTOR_ERROR,  // 翻盖电机状态
  LORA_ERROR,              // lora模组
  BMS_ERROR,               // 电池模组
  IMU_ERROR,               // imu
  MCU_SELF_CHECK_DONE = 2999,  // mcu自检完成

  // slam_node code
  SLAM_INIT_ERROR = 3000,
  ENCODER_CHECK_ERROR = 3001,
  IMU_CHECK_ERROR = 3002,
  ENCODER_IMU_ALIGN_ERROR = 3003,
  SLAM_INIT_DONE = 3999,

  // planing_node error_code
  PLANING_NODE_ERROR = 4000,
  PLANING_NODE_POSE_CHECK_ERROR = 4001,
  PLANING_NODE_JAM_ERROR = 4002,
  PLANING_NODE_MAX = 4999,

  // EventTracking 埋点
  Event_Tracking_INIT_ERROR = 10000,  // 例子，开发者可以根据自己的需求添加相应的错误码
} status_code;


// 错误码对应的错误信息
const std::unordered_map<status_code, std::string> status_code_map = {
  {SOC_IMU_INIT_ERROR, "soc_Imu init errro"},
  {SOC_IMU_DATA_ERROR, "soc_Imu data error"},
  {MAG_INIT_ERROR, "magnetometer init error"}
};

// 获取错误码对应的错误信息
inline std::string get_status_code_info(status_code code) {
  if (status_code_map.find(code) == status_code_map.end()) {
    return "unknown error code";
  }
  return status_code_map.at(code);
}
#endif  // SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_
