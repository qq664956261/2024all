/**
 * *****************************************************************************
 * @file        machine_state.h
 * @brief
 * @author      vicwang (vic@yardbot.net)
 * @date        2024-02-01
 * @copyright   HJ technology
 * *****************************************************************************
 */
#ifndef _MACHINE_STATE_H_
#define _MACHINE_STATE_H_
#include "enum.h"
#include <string>
#include <unordered_map>

namespace machine {
smart_enum_class(MachineState,
                 booting,          // 启动中
                 factoryResetting, // 恢复出厂设置中
                 updating,         // 软件更新中
                 shuttingDown,     // 关机中
                 callingBack,      // 召回中(无桩)
                 backingCharge,    // 回充中(有桩)
                 docking,          // 与基站对接中(有桩)
                 undocking,        // 与基站脱离中(有桩)
                 exitingCharge, // 退出充电中(包含整个下水动作)(有桩)
                 charging,      // 充电中
                 running,       // 运行中
                 suspending,     // 暂停中
                 alarming,       // 报警中
                 mapBuilding,    // 地图构建中
                 mooring,        // 停泊中(无桩)
                 standingby,     // 待机中(离开了泳池)(无桩)
                 manuMoving,     // 手动移动中
                 idel,           // 空闲中(未离开泳池)(无桩)
                 outOfWatering,  // 出水中
                 exitStandingby, // 退出待机中(有桩)
                 sleep,          // 睡眠中
                 pairNetwork,    // 配网中
                 remoteCtrl,     // 云平台远程控制中
);

smart_enum_class(MachineEvent,
                 phyManuMove,  // 手动移动(遥控器)
                 virManuMove,  // 虚拟移动(APP)
                 callback,     // 召回
                 backCharge,   // 回充
                 toCharge,     // 转充电
                //  charge,       // 充电
                 toExitCharge, // 退出充电
                 update,       // 软件更新
                 autoRun,      // 自动运行
                 suspend,      // 暂停
                 resume,       // 恢复
                 abort,        // 中止
                 toAlarm,      // 报警
                 clearAlarm,   // 清除报警
                 buildMap,     // 构建地图
                 // configWifi,          // 配置wifi
                 // configBle,           // 配置蓝牙
                 toMoored,     // 停泊
                 reboot,       // 重启
                 resetFactory, // 恢复出厂设置
                 shutdown,     // 关机
                 toStandby,    // 进入待机
                 outOfWater,   // 出水
                 toIdel,       // 进入空闲
                 toSleep,      // 进入睡眠
                 toPairNetwork,// 配网
                 toRemoteCtrl  // 云平台远程控制
);

static std::unordered_map<MachineState, std::string> state_enum_to_str_map = {
    {MachineState::booting, "booting"},
    {MachineState::factoryResetting, "factoryResetting"},
    {MachineState::updating, "updating"},
    {MachineState::shuttingDown, "shuttingDown"},
    // {MachineState::bleConfiging, "bleConfiging"},
    // {MachineState::wifiConfiging, "wifiConfiging"},
    {MachineState::callingBack, "callingBack"},
    {MachineState::backingCharge, "backingCharge"},
    {MachineState::docking, "docking"},
    {MachineState::charging, "charging"},
    {MachineState::exitingCharge, "exitingCharge"},
    {MachineState::undocking, "undocking"},
    {MachineState::running, "running"},
    {MachineState::suspending, "suspending"},
    {MachineState::alarming, "alarming"},
    {MachineState::mapBuilding, "mapBuilding"},
    {MachineState::mooring, "mooring"},
    {MachineState::standingby, "standingby"},
    //   {MachineState::exitStandingby, "exitStandingby"},
    {MachineState::manuMoving, "manuMoving"},
    {MachineState::idel, "idel"},
    {MachineState::sleep, "sleep"},
    {MachineState::pairNetwork, "pairNetwork"},
    //   {MachineState::outOfWatering, "outOfWatering"}
    {MachineState::remoteCtrl, "remoteCtrl"}
};

static std::unordered_map<MachineEvent, std::string> event_enum_to_str_map = {
    {MachineEvent::phyManuMove, "phyManuMove"},
    {MachineEvent::virManuMove, "virManuMove"},
    {MachineEvent::callback, "callback"},
    {MachineEvent::toCharge, "toCharge"},
    // {MachineEvent::charge, "charge"},
    {MachineEvent::toExitCharge, "toExitCharge"},
    {MachineEvent::update, "update"},
    {MachineEvent::autoRun, "autoRun"},
    {MachineEvent::suspend, "suspend"},
    {MachineEvent::resume, "resume"},
    {MachineEvent::abort, "abort"},
    {MachineEvent::toAlarm, "toAlarm"},
    {MachineEvent::clearAlarm, "clearAlarm"},
    {MachineEvent::buildMap, "buildMap"},
    // {MachineEvent::configWifi, "configWifi"},
    // {MachineEvent::configBle, "configBle"},
    {MachineEvent::toMoored, "toMoored"},
    {MachineEvent::reboot, "reboot"},
    {MachineEvent::resetFactory, "resetFactory"},
    {MachineEvent::shutdown, "shutdown"},
    {MachineEvent::toStandby, "toStandby"},
    {MachineEvent::outOfWater, "outOfWater"},
    {MachineEvent::toIdel, "toIdel"},
    {MachineEvent::toSleep, "toSleep"},
    {MachineEvent::toPairNetwork, "toPairNetwork"},
    {MachineEvent::toRemoteCtrl, "toRemoteCtrl"}};

static std::unordered_map<std::string, MachineState> state_str_to_enum_map = {
    {"booting", MachineState::booting},
    {"factoryResetting", MachineState::factoryResetting},
    {"updating", MachineState::updating},
    {"shuttingDown", MachineState::shuttingDown},
    // {"bleConfiging", MachineState::bleConfiging},
    // {"wifiConfiging", MachineState::wifiConfiging},
    {"callingBack", MachineState::callingBack},
    {"backingCharge", MachineState::backingCharge},
    {"docking", MachineState::docking},
    {"charging", MachineState::charging},
    {"exitingCharge", MachineState::exitingCharge},
    {"undocking", MachineState::undocking},
    {"running", MachineState::running},
    {"suspending", MachineState::suspending},
    {"alarming", MachineState::alarming},
    {"mapBuilding", MachineState::mapBuilding},
    {"mooring", MachineState::mooring},
    {"standingby", MachineState::standingby},
    //   {"exitStandingby", MachineState::exitStandingby},
    {"manuMoving", MachineState::manuMoving},
    {"idel", MachineState::idel},
    {"sleep", MachineState::sleep},
    {"pairNetwork", MachineState::pairNetwork},
    //   {"outOfWatering", MachineState::outOfWatering}
    {"remoteCtrl", MachineState::remoteCtrl}
};

static std::unordered_map<std::string, MachineEvent> event_str_to_enum_map = {
    {"phyManuMove", MachineEvent::phyManuMove},
    {"virManuMove", MachineEvent::virManuMove},
    {"callback", MachineEvent::callback},
    {"toCharge", MachineEvent::toCharge},
    // {"charge", MachineEvent::charge},
    {"toExitCharge", MachineEvent::toExitCharge},
    {"update", MachineEvent::update},
    {"autoRun", MachineEvent::autoRun},
    {"suspend", MachineEvent::suspend},
    {"resume", MachineEvent::resume},
    {"abort", MachineEvent::abort},
    {"toAlarm", MachineEvent::toAlarm},
    {"clearAlarm", MachineEvent::clearAlarm},
    {"buildMap", MachineEvent::buildMap},
    // {"configWifi", MachineEvent::configWifi},
    // {"configBle", MachineEvent::configBle},
    {"toMoored", MachineEvent::toMoored},
    {"reboot", MachineEvent::reboot},
    {"resetFactory", MachineEvent::resetFactory},
    {"shutdown", MachineEvent::shutdown},
    {"toStandby", MachineEvent::toStandby},
    {"outOfWater", MachineEvent::outOfWater},
    {"toIdel", MachineEvent::toIdel},
    {"toSleep", MachineEvent::toSleep},
    {"toPairNetwork", MachineEvent::toPairNetwork},
    {"toRemoteCtrl", MachineEvent::toRemoteCtrl}};

static std::unordered_map<MachineState, MachineEvent> state_to_event_map = {
    {MachineState::booting, MachineEvent::reboot},
    {MachineState::factoryResetting, MachineEvent::resetFactory},
    {MachineState::updating, MachineEvent::update},
    {MachineState::shuttingDown, MachineEvent::shutdown},
    {MachineState::callingBack, MachineEvent::callback},
    // {MachineState::backingCharge, MachineEvent::toCharge},
    {MachineState::docking, MachineEvent::autoRun},
    {MachineState::charging, MachineEvent::toCharge},
    {MachineState::exitingCharge, MachineEvent::toExitCharge},
    {MachineState::undocking, MachineEvent::autoRun},
    {MachineState::running, MachineEvent::autoRun},
    {MachineState::suspending, MachineEvent::suspend},
    {MachineState::alarming, MachineEvent::toAlarm},
    {MachineState::mapBuilding, MachineEvent::buildMap},
    {MachineState::mooring, MachineEvent::toMoored},
    {MachineState::standingby, MachineEvent::toStandby},
    //   {MachineState::exitStandingby, MachineEvent::toStandby},
    {MachineState::manuMoving, MachineEvent::phyManuMove},
    {MachineState::idel, MachineEvent::toIdel},
    {MachineState::sleep, MachineEvent::toSleep},
    {MachineState::pairNetwork, MachineEvent::toPairNetwork},
    //   {MachineState::outOfWatering, MachineEvent::outOfWater}
    {MachineState::remoteCtrl, MachineEvent::toRemoteCtrl}
};

static std::unordered_map<MachineEvent, MachineState> event_to_state_map = {
    {MachineEvent::reboot, MachineState::booting},
    {MachineEvent::resetFactory, MachineState::factoryResetting},
    {MachineEvent::update, MachineState::updating},
    {MachineEvent::shutdown, MachineState::shuttingDown},
    {MachineEvent::callback, MachineState::callingBack},
    {MachineEvent::backCharge, MachineState::backingCharge},
    // {MachineEvent::toCharge, MachineState::backingCharge},
    // {MachineEvent::autoRun, MachineState::docking},
    {MachineEvent::toCharge, MachineState::charging},
    {MachineEvent::toExitCharge, MachineState::exitingCharge},
    // {MachineEvent::autoRun, MachineState::undocking},
    {MachineEvent::autoRun, MachineState::running},
    // {MachineEvent::suspend, MachineState::suspending},
    {MachineEvent::abort, MachineState::idel},
    {MachineEvent::toAlarm, MachineState::alarming},
    {MachineEvent::clearAlarm, MachineState::idel},
    // {MachineEvent::buildMap, MachineState::mapBuilding},
    {MachineEvent::toMoored, MachineState::mooring},
    {MachineEvent::toStandby, MachineState::standingby},
    //   {MachineEvent::toStandby, MachineState::exitStandingby},
    {MachineEvent::phyManuMove, MachineState::manuMoving},
    {MachineEvent::toIdel, MachineState::idel},
    {MachineEvent::toSleep, MachineState::sleep},
    {MachineEvent::toPairNetwork, MachineState::pairNetwork},
    {MachineEvent::outOfWater, MachineState::standingby},
    //   {MachineEvent::outOfWater, MachineState::outOfWatering}
    {MachineEvent::toRemoteCtrl, MachineState::remoteCtrl}
};

enum class ReportAppStatue {
  IDLE = 0,           //初始化状态
  CLEANNING = 1,      //清扫中
  CHARGING = 2,       //充电中
  BATTERY_FULL = 3,   //电池已充满
  BACKING_CHARGE = 4, //回充中
  MOORING = 5,        //停泊中
  CALLING_BACK = 6,   //召回中
  STANDBY = 7,        //待机中
  SLEEP = 8,          //休眠中
  ALARM = 9,          //报警中
  OTA_UPDATING = 10,   // OTA更新中
  PAIR_NETWORK = 11   // 配网中

};

} // namespace machine

#endif // _MACHINE_STATE_H_