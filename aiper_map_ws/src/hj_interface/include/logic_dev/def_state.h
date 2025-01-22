// @File: def_state.h
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Author: Harvey
// Date: 2024/03/25
#ifndef DEF_STATE_H_
#define DEF_STATE_H_
#include <unordered_map>
#include <tuple>
#include <time.h>
#include "logic_dev/machine_state.h"

//遥控器按键定义
enum class PhyCtlKey {
  KEY_IDLE  = -1,
  KEY_STOP  = 0,
  KEY_LEFT  = 1,
  KEY_RIGHT = 2,
  KEY_DOWN  = 4,
  KEY_UP    = 8,
  KEY_ENTER = 16,     //任务模式
  KEY_SNORKEL = 32,    //浮潜
  KEY_RECALL = 128

};

enum class SnorkelKey {
  KEY_IDLE = -1,
  KEY_INFLATABLE = 0, //全充气
  KEY_OUTFLATABLE,    //全放气
  KEY_LEFT_INFLATABLE, //左充气
  KEY_RIGHT_INFLATABLE //右充气
};

static std::unordered_map<PhyCtlKey, std::tuple<int32_t, int32_t, uint8_t, uint8_t>> phy_ctl_key_to_motor_map = {
  {PhyCtlKey::KEY_STOP,  {0, 0, 0, 0}},
  {PhyCtlKey::KEY_LEFT,  {-200, 200, 38, 38}},
  {PhyCtlKey::KEY_RIGHT, {200, -200, 38, 38}},
  {PhyCtlKey::KEY_DOWN,  {-200, -200, 38, 38}},
  {PhyCtlKey::KEY_UP,    {200, 200, 38, 38}},
};

enum class TaskType {
  FAST,       //快速任务
  TIMMING,    //计时任务     
  PERIODIC,   //周期任务
  MANUAL,      //手动任务
  INTERVAL    //间隔任务
};

enum class PeriodicType {
  WEEKDAY = 1,  //星期
  INTERVAL      //间隔
};


enum class CleanMode {
  VAR_FREQ = 1,   //变频
  DEEP,           //深度
  STANDARD       //标准
};

enum class AreaType {
  NROMAL = 0,   //普通模式
  SMART = 1,     //智能模式 11+12+13
  UNDER_WATER = 2  //水下模式 12+13
};

enum class CleanArea {
  IDEL = 0,  
  AREA1 = 11,  //水面
  AREA2,      //池底
  AREA3,      //池壁
  AREA4       //水线
#ifdef FOR_MULTI_PLATFORM_CLEAN
  ,AREA5      //多平台清洁
#endif

};

enum class BottomRunMode {
  CROSS_BOW = 1,        //十字弓
  ADAPTIVE_BOW  = 2     //自适应弓
};

static std::unordered_map<CleanArea, std::string> clean_area_map = {
  {CleanArea::AREA1, "Water Surface"},
  {CleanArea::AREA2, "Pool Bottom"},
  {CleanArea::AREA3, "Pool Wall"},
  {CleanArea::AREA4, "Water Line"}
#ifdef FOR_MULTI_PLATFORM_CLEAN
  ,{CleanArea::AREA5, "Multi Platform"}
#endif
};

enum class CleanPriority {
  FIRSTLY = 1,  //优先
  SECONDLY,     //次优先
  THIRDLY,      //再优先
  FOURTHLY      //最后优先
};

enum class TaskMode {
  None,       //init
  Interval,   //间隔
  ByWeekday     //星期
};

using CLEAN_AREAS = std::vector<std::tuple<CleanArea, int32_t, int32_t>>;   //std::tuple<清洁区域，次数，时间>
struct TaskInfo {
  std::string name_id_;
  TaskType task_type_;
  std::tm start_tm_;
  AreaType smart_;      // 0-普通模式 1-智能模式 2-水下模式
  CLEAN_AREAS clean_areas_;   //std::tuple<清洁区域, 清洁次数, 清洁时间>
  int32_t run_time_;  //单位：分钟, 针对计时任务
  int32_t clean_mode_;  // 1-变频 2-标准 3-深度
};

enum class TaskCmd {
  BUILD_MAP = 11,   //建图
  LOCALIZE = 12,    //定位

  CALL_BACK = 21,    //召回
  BACK_CHARGE = 22,  //回充

  REQ_STATE = 31,     //状态请求
  CLEAN_MODE = 36,    //清洁模式

  START = 100,        //开始
  PAUSE = 101,        //暂停
  RESUME,             //继续
  STOP,               //停止
  FAIL                //失败
};

enum class TaskStatus {
  IDEL,     //空闲
  RUNNING,  //运行中
  PAUSE,    //暂停
  STOP      //停止
};

enum class NaviState {
  IDEL = 0,      //空闲
  WORKING = 32,  //运行中
  STANDBY = 33   //待机
};

// enum class ManualTaskMode {
//   SURFACE_WATER = 1,   //水面
//   POOL_BOTTOM = 2,     //池底
//   POOL_WALL = 3,       //池壁
//   SMART =4             //智能
// };

enum class ManualTaskMode {
  SMART = 1,     //水下
  POOL_BOTTOM = 2,     //池底
  POOL_WALL = 3,      //水线
  WATER_LINE =4         //池壁
};

enum class X9ManualTaskMode
{
  SMART = 1,         // 智能
  POOL_BOTTOM = 2,   // 池底
  UNDER_WATER = 3,   // 水下(池底+池壁)
  SURFACE_WATER = 4  // 智能
};

enum class T1ProManualTaskMode
{
  SMART = 1,       // 智能
  POOL_BOTTOM = 2, // 池底
  POOL_WALL = 3,   // 池壁
  WATER_LINE = 4   // 水线
};

//向slam&navi发送的所有请求
enum class ToSlamOrNaviReq {
  IDEL        = 0,      //初始化状态
  BUILD_MAP   = 1,      //建图
  LOCALIZE    = 2,      //定位
  AREA1       = 11,     //水面
  AREA2       = 12,     //池底
  AREA3       = 13,     //池壁
  AREA4       = 14,     //水线
  POOL_BOTTOM_TASK = 15,  //池底任务
  NAVI_EDGE        = 16,     //建图/重定位延边运动
  NAVI_LIMIT_EDGE  = 17,     //极限延边
  READY_POSTURE    = 18,     //姿态调整
  CLEAN_MODE       = 19,     //清洁模式
  CALLBACK_EDGE    = 20,     //召回延边
  CALL_BACK        = 21,     //召回
  BACK_CHARGE      = 22,     //回充
  NEAREST_EDGE_CALLBACK = 25,//最近边召回
  ENTRY_POINT_CALLBACK = 26, //入水点召回
  ENTRY_EDGE_CALLBACK = 27,  //入水侧召回
  CALLBACK_RELOCATION = 28,  //召回重定位
  MOORING          = 29,     //停泊
  SLAM_STATE       = 31,     //SLAM状态请求 
  NAVI_STATE       = 32,     //NAVI状态请求
  GET_CLEAN_RECORD = 35,     //获取清洁记录信息
  STOP_BUILD_MAP   = 103,    //停止建图
  STOP_LOCALIZE    = 104,    //停止定位
  PAUSE_CLEANING   = 111,    //暂停清扫
  RESUME_CLEANING  = 112,    //继续清扫
  STOP_CLEANING    = 113,    //停止清扫
  STOP_EDGE        = 115,    //停止延边
  STOP_LIMIT_EDGE  = 116,    //停止极限延边
  STOP_READY_POSTURE = 117,  //停止姿态调整
  STOP_CALL_BACK   = 123,    //停止召回
  STOP_BACK_CHARGE = 124,    //停止回充
  STOP_MOORING     = 125    //停止停泊
#ifdef FOR_MULTI_PLATFORM_CLEAN
  ,MUILTI_PLATFORM_CLEAN = 200      //多平台清洁
#endif
  
};

//slam&navi返回的工作状态
enum class SlamAndNaviState {
  IDLE            = 0,    //初始化状态
  SLAM_STANDBY    = 1,    //slam待机
  SLAM_BUILDING   = 2,    //slam建图中
  SLAM_LOCALIZING = 3,    //slam定位中
  NAVI_STANDBY    = 11,   //navi待机
  NAVI_CLEANNING  = 12,    //navi清扫中
  NAVI_EDGEING    = 16,    //navi延边中
  NAVI_LIMIT_EDGEING = 17,  //navi极限延边中
  NAVI_READY_POSTURE = 18,  //navi姿态调整中
  NAVI_CALLING_BACK  = 21,  //navi召回中
  NAVI_BACK_CHARGING = 22   //navi回充中
};

//slam&navi返回工作的对象TaskMode
enum class FromSlamOrNaviResp {
  IDLE            = 0,      //初始化状态
  BUILD_MAP       = 1,      //建图
  LOCALIZE        = 2,      //定位
  CLEANNING       = 11,     //清扫
  NAVI_EDGE       = 16,     //延边
  NAVI_LIMIT_EDGE = 17,     //极限延边
  READY_POSTURE   = 18,     //姿态调整
  CALL_BACK       = 21,     //召回
  BACK_CHARGE     = 22,     //回充
  NEAREST_EDGE_CALLBACK = 25,//最近边召回
  ENTRY_POINT_CALLBACK = 26, //入水点召回
  ENTRY_EDGE_CALLBACK = 27,  //入水侧召回
  MOORING = 29              //停泊
};

//slam&navi返回工作的结果
enum class SlamOrNaviRespResult {
  IDLE                     = 0,      //初始化状态
  SLAM_BUILD_SUCCESS       = 1,      //SLAM建图成功
  SLAM_BUILD_FAIL          = 2,      //SLAM建图失败 -- 后期定义失败的具体错误码
  SLAM_LOCALIZE_SUCCESS    = 11,     //SLAM定位成功
  SLAM_LOCALIZE_FAIL       = 12,     //SLAM定位失败 -- 后期定义失败的具体错误码
  NAVI_CLEAN_SUCCESS       = 21,     //NAVI清扫成功
  NAVI_CLEAN_FAIL          = 22,     //NAVI清扫失败 -- 后期定义失败的具体错误码
  NAVI_EDGE_STOP           = 26,     //NAVI延边停止
  NAVI_LIMIT_EDGE_STOP     = 27,     //NAVI极限延边停止
  NAVI_POSTURE_SUCCESS     = 31,     //NAVI姿态调整成功
  NAVI_POSTURE_FAIL        = 32,     //NAVI姿态调整失败 -- 后期定义失败的具体错误码
  NAVI_WITHOUT_MAP         = 33,     //NAVI转无图召回
  NAVI_CALL_BACK_SUCCESS   = 41,     //NAVI召回成功
  NAVI_CALL_BAC_FAIL       = 42,     //NAVI召回失败 -- 后期定义失败的具体错误码
  NAVI_MOORING_SUCCESS     = 43,     //NAVI停泊成功
  NAVI_MOORING_FAIL        = 44,     //NAVI停泊失败 -- 后期定义失败的具体错误码
  NAVI_BACK_CHARGE_SUCCESS = 51,     //NAVI回充成功
  NAVI_BACK_CHARGE_FAIL    = 52      //NAVI回充失败 -- 后期定义失败的具体错误码
};

#endif // DEF_STATE