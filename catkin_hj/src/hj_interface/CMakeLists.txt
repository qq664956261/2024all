cmake_minimum_required(VERSION 3.0.2)
project(hj_interface)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PROJECT_NUMBER "x9" CACHE STRING "project number, such as 2312")
message("Current project number is ${PROJECT_NUMBER}")
add_definitions(-DPROJECT_NUMBER=${PROJECT_NUMBER})

set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
add_definitions(-DNODE_NAME="${PROJECT_NAME}")
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp std_msgs geometry_msgs)

add_message_files(
  FILES
  Mag.msg
  Bat.msg
  Encoder.msg
  Imu.msg
  SocImu.msg
  Nav.msg
  Depth.msg
  Kbd.msg
  Turbidity.msg
  Ray.msg
  Ultra.msg
  Pose.msg
  AirBag.msg
  FlipCover.msg
  NaviWheel.msg
  SelfCleanMotor.msg
  SteerAndPump.msg
  # HealthCheckCode.msg
  UploadIot.msg
  StateDeal.msg
  AbnormalState.msg
  CleanAreas.msg
  PeriodicWeekdayTask.msg
  PeriodicIntervalTask.msg
  PeriodicTaskPkg.msg 
  SignalTask.msg
  Chat.msg
  PumpAndSteer.msg
  X6Pump.msg
  AppMsg.msg
  IotReturn.msg
  TurbineMotor.msg
  DockSignal.msg
  WorkResult.msg
  OtaPackData.msg 
  OtaUpgradeData.msg 
  subPackData.msg 
  OtaUpgradeStatus.msg
  CleanArea.msg
  ElecMotorCur.msg
  ImuWorkModel.msg
  AppMsg.msg
  IotReturn.msg
  StationUltra.msg
  AppData.msg
  WifiSet.msg
  ActionInfo.msg
  TempHumidity.msg
  Fall.msg
  AppOnlineType.msg
)

add_service_files(
  FILES
  HealthCheckCode.srv
  IotShadow.srv
  NaviAction.srv
  SlamAction.srv
  SlamNaviWorkResult.srv
  MagCalibAction.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

add_custom_target(
   install_inc
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/include/* ${CMAKE_CURRENT_SOURCE_DIR}/include
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/
)
add_custom_target(
   delete_inc
  COMMAND
  rm -rf  ${CMAKE_CURRENT_SOURCE_DIR}/include/hj_interface
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/
)
