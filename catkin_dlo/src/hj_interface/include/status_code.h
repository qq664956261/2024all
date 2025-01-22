#ifndef SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_
#define SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_

typedef enum {
  // collect_node error code
  IMU_INIT_ERROR = 1000,                   // IMU初始化失败
  IMU_READ_ERROR = 1001,                   // IMU获取数据失败
  MAG_INIT_ERROR = 1002,                   // MAG初始化失败
  MAG_READ_ERROR = 1003,                   // MAG获取数据失败
  TUR_INIT_ERROR = 1004,                   // TUR初始化失败
  TUR_READ_ERROR = 1005,                   // TUR获取数据失败
  TUR_WRITE_ERROR = 1006,                  // TUR写数据失败
  ULSOUND_INTI_ERROR = 1007,               // uls初始化失败
  ULSOUND_FRONT_USS_READ_ERROR = 1008,     // 三合一超声读数据失败
  ULSOUND_FRONT_TOF_READ_ERROR = 1009,     // 三合一tof读取数据失败
  ULSOUND_SIDE_MID_READ_ERROR = 1010,      // 侧边中间超声读取数据失败
  ULSOUND_SIDE_BACK_READ_ERROR = 1011,     // 侧边后面超声读取数据失败
  WF5803_INIT_ERROR = 1012,                // 压力计初始化失败
  WF5803_READ_ERROR = 1013,                // 压力计读数据失败
  WF5803_WRITE_ERROR = 1014,               // 压力计写数据失败
  COLLECT_NODE_MAX,                        // collect node error code max


  // mcu/midleware_ndoe error code
  MCU_INIT_ERROR = 2000,

  // slam_node code
  SLAM_INIT_ERROR = 3000,

  // planing_node error_code
  PLANING_NODE_ERROR = 4000,
} status_code;

#endif  // SRC_HJ_INTERFACE_INCLUDE_STATUS_CODE_H_
