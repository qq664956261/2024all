#ifndef _APPINTERFACE_H_
#define _APPINTERFACE_H_
/*=============================================Report==================================================*/
#define REP_CMD_DEVICE_INFO "DevInfoReport"  // 获取 上报设备信息 开机上报一次
#define REP_CMD_OP_INFO "OpInfoReport"    // 获取&上报运行信息
#define REP_CMD_IN_WATER "InwaterReport"  // 在水信息
#define REP_CMD_GET_RUBBISH_BOX_STATUS \
  "GetRubbishBoxStatusReport"                 // 垃圾篓状态
#define REP_CMD_WQS "WQSReport"               // 获取水质传感器数据
#define REP_CMD_OTA_STATUS "OtaStatusReport"  // OTA进度上报
#define REP_CMD_CLOUD_OTA_RESULT "OtaResultReport"  // 上报云 OTA结果
#define OP_INFO_REPORT "OpInfoReport"
#define REP_REMOTE_CMD_FLUSH_STATUS "FlushStatusReport"  // 远程控制 状态更新上报
#define REP_REQUEST_REMOTE_AUTH "authReport"  // 远程控制授权
/*=============================================Request==================================================*/
#define REQ_CMD_OTA_RUN "UrlOta"  // URL升级
#define REQ_CMD_DEVICE_INFO "DevInfo"  // 获取 上报设备信息 开机上报一次
#define REQ_CMD_OP_INFO "OpInfo"    // 获取&上报运行信息
#define REQ_CMD_IN_WATER "Inwater"  // 在水信息
#define REQ_CMD_GET_RUBBISH_BOX_STATUS "GetRubbishBoxStatus"  // 垃圾篓状态
#define REQ_CMD_WQS "WQS"  // 获取水质传感器数据
#define REQ_REMOTE_CMD_FLUSH_STATUS "FlushStatus"  // 远程控制 状态更新
#define REQ_REMOTE_CMD_SYSTEM_RESET "SystemReset"  // 远程控制 模组重启
constexpr const char* kReqCmdInNetCfg = "InNetConfig";
constexpr const char* kReqCmdShutdown = "Shutdown";
constexpr const char* kReqCmdReset = "Reset";
constexpr const char* kReqCmdLifeTime = "LifeTime";
constexpr const char* kReqGetEntryPoint = "GetEntryPoint";
constexpr const char* kReqSetEntryPoint = "SetEntryPoint";
constexpr const char* kReqGetAirBagStatus = "GetAirBagStatus";
constexpr const char* kAirBagStatusReport = "AirBagStatusReport";
#define REQ_CMD_STOP_ACTION "StopAction"  // 停止动作

/*======================================远程控制Request==================================================*/
#define REQ_REQUEST_REMOTE_AUTH "auth"  // 远程控制授权
#define REQ_REQUEST_REMOTE_UNLOCK "unlock"    // 远程控制 解除控制请求
#endif                                  // _APPINTERFACE_H_
