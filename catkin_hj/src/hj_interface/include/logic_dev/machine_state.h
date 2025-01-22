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
#include <unordered_map>
#include <string>
#include "enum.h"

namespace machine
{
  smart_enum_class(MachineState,
                   booting,          // 启动中
                   factoryResetting, // 恢复出厂设置中
                   updating,         // 软件更新中
                   shuttingDown,     // 关机中
                   callingBack,    // 召回中(无桩)
                   backingCharge,  // 回充中(有桩)
                   docking,        // 与基站对接中(有桩)
                   undocking,      // 与基站脱离中(有桩)
                   exitingCharge,  // 退出充电中(包含整个下水动作)(有桩)
                   charging,       // 充电中
                   running,        // 运行中
                   suspending,     // 暂停中
                   alarming,       // 报警中
                   mapBuilding,    // 地图构建中
                   mooring,        // 停泊中(无桩)
                   standingby,     // 待机中(离开了泳池)(无桩)
                   manuMoving,     // 手动移动中
                   idel,           // 空闲中(未离开泳池)(无桩)
                   outOfWatering,  // 出水中
                   exitStandingby, // 退出待机中(有桩)
  );

  smart_enum_class(MachineEvent,
                   phyManuMove,  // 手动移动(遥控器)
                   virManuMove,  // 虚拟移动(APP)
                   callback,     // 回充
                   toCharge,     // 转充电
                   charge,       // 充电
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
                   toIdel        // 进入空闲
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
      {MachineState::idel, "idel"}
    //   {MachineState::outOfWatering, "outOfWatering"}
    };

  static std::unordered_map<MachineEvent, std::string> event_enum_to_str_map = {
      {MachineEvent::phyManuMove, "phyManuMove"},
      {MachineEvent::virManuMove, "virManuMove"},
      {MachineEvent::callback, "callback"},
      {MachineEvent::toCharge, "toCharge"},
      {MachineEvent::charge, "charge"},
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
      {MachineEvent::reboot, "reboot"},
      {MachineEvent::resetFactory, "resetFactory"},
      {MachineEvent::shutdown, "shutdown"},
      {MachineEvent::toStandby, "toStandby"},
      {MachineEvent::outOfWater, "outOfWater"},
      {MachineEvent::toIdel, "toIdel"}};

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
      {"idel", MachineState::idel}
    //   {"outOfWatering", MachineState::outOfWatering}
    };

  static std::unordered_map<std::string, MachineEvent> event_str_to_enum_map = {
      {"phyManuMove", MachineEvent::phyManuMove},
      {"virManuMove", MachineEvent::virManuMove},
      {"callback", MachineEvent::callback},
      {"toCharge", MachineEvent::toCharge},
      {"charge", MachineEvent::charge},
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
      {"reboot", MachineEvent::reboot},
      {"resetFactory", MachineEvent::resetFactory},
      {"shutdown", MachineEvent::shutdown},
      {"toStandby", MachineEvent::toStandby},
      {"outOfWater", MachineEvent::outOfWater},
      {"toIdel", MachineEvent::toIdel}};

  static std::unordered_map<MachineState, MachineEvent> state_to_event_map = {
      {MachineState::booting, MachineEvent::reboot},
      {MachineState::factoryResetting, MachineEvent::resetFactory},
      {MachineState::updating, MachineEvent::update},
      {MachineState::shuttingDown, MachineEvent::shutdown},
      {MachineState::callingBack, MachineEvent::callback},
      {MachineState::backingCharge, MachineEvent::toCharge},
      {MachineState::docking, MachineEvent::autoRun},
      {MachineState::charging, MachineEvent::charge},
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
      {MachineState::idel, MachineEvent::toIdel}
    //   {MachineState::outOfWatering, MachineEvent::outOfWater}
    };

} // namespace machine

#endif // _MACHINE_STATE_H_