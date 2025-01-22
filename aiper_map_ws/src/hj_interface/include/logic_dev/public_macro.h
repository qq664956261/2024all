#ifndef PUBLIC_MACRO_H
#define PUBLIC_MACRO_H

#define BACKUP_PATH              "/data/hj/config/"
#define LOGIC_DIR                "/userdata/logic/"
#define CLOUD_VERSION_FILE_PATH       LOGIC_DIR    "cloud_version.json"
#define PARA_CONFIG_FILE_PATH         LOGIC_DIR    "paraConfig.json"
#define BACKUP_PARA_CONFIG_FILE_PATH  BACKUP_PATH  "paraConfig.json" 
#define CLEAN_RECORD_DIR              LOGIC_DIR    "clean_record"
// #define CLEAN_RECORD_DIR              "/tmp/logic/clean_record"
#define PATH_BTN_NEW_MARK "/userdata/logic/btn_new.mark"
#define TIME_ZONE_FILE_PATH           "/userdata/.root/etc/tzm.json"
#define PATH_SAVE_AIR_BAG_STATUS "/userdata/logic/airBagStatus.json"
/******************保存清洁记录的最大天数*********************/
const int32_t CLEAN_RECORD_MAX_DAYS = 28;
#define MAP_DIR                    "/userdata/hj/maps/"
#define MAP_FILE_PATH               MAP_DIR      "point_cloud.bin"
#define MAP_FILE_DIR                MAP_DIR      "map/"

/******************开机后加载的最后一个任务路径*********************/
const int32_t CLEAN_AREA_DIFF_BETWEEN_APP_AND_MACHINE = 10;  //清洁区域之间的间隔
#define LAST_TASK_FILE_PATH         LOGIC_DIR    "last_task.json"
// #define LAST_TASK_FILE_PATH         "/tmp/last_task.json"

/*****************出入水检测宏*********************/
#define USE_CHECK_WATER_SENSOR

/******************coredump测试宏*********************/
// #define DEBUG_COREDUMP

/******************演示宏*********************/
// #define FOR_2024_7_2_DEMO

/******************多平台宏*********************/
#define FOR_MULTI_PLATFORM_CLEAN

/*************区分工厂模式和大货模式的字段文件来源*********/
#define DEVICE_INFO_FILE     "/tmp/devInfo.json"
const std::string NORMAL_RUN = "normal";
const std::string FACTORY_RUN = "fac";

/******************OTA*********************/
#define DELAYTIMER_MAX_FILE "/userdata/logic/delaytimer_max.json"
/******************water_sensor*********************/
#define WATER_SENSOR_CHECK_FILE "/userdata/logic/water_sensor.json"
#define NO_ENV_WATER_REVERSE_PATH "/userdata/logic/no_env_water.mark"
//
#endif // PUBLIC_MACRO_H