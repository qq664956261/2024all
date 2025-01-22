#ifndef _ST_DEFINE_H_
#define _ST_DEFINE_H_
#include <iostream>
// push
#define OTA_RESULT_EVENT "otaResultEvent"
#define MACHINE_ON_EVENT "machineOnEvent"
#define BUTTON_EVENT "buttonEvent"
#define BATTERY_EVENT "batteryEvent"
#define PUMP_CURRENT_VARIBALE_EVENT "pumpCurrentEvent"

constexpr const char* kTimeoutShutdownEvent = "machineOffOvertimeEvent";

namespace st_define {
struct OtaUpgrade {
  std::string event;
  std::string cloudVersion;
  std::string robotVersion;
  uint8_t updateType;
  uint64_t otaTimeStamp;
  struct Upgrade {
    uint64_t beginTimeStamp;
    uint64_t endTimeStamp;
    uint8_t result;  // 0: 成功，1:失败
    std::string errorMsg;
  } download, update;
};

struct MachineOn {
  double onTime;
  uint64_t timeStamp;
  std::string robotVersion;
};

struct Button {
  uint8_t buttonType;
  uint8_t buttonStatus;
  std::string robotVersion;
  uint64_t timeStamp;
};

struct Battery {
  uint64_t recycle;
  uint64_t capacity;
  uint64_t timeStamp;
  std::string robotVersion;
};

struct OutInwater {
  uint64_t timeStamp;
  uint8_t status;
  struct Current_Analysis {
    uint32_t left_sum;
    uint32_t right_sum;
    uint32_t left_sizes;
    uint32_t right_sizes;
    uint32_t left_avg;
    uint32_t right_avg;
    uint32_t delay_sleep;
  } current_analysis;
  struct Current_Now
  {
    uint32_t left_current;
    uint32_t right_current;
  } now_cur;
  uint8_t analysis_status;
};

}  // namespace st_define

#endif  // _ST_DEFINE_H_
