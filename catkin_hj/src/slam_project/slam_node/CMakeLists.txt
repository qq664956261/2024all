cmake_minimum_required(VERSION 3.0.2)
set(NODE_NAME slam_node)
project(${NODE_NAME})
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PROJECT_NUMBER "x9" CACHE STRING "project number, such as 2312")
message("Current project number is ${PROJECT_NUMBER}")
add_definitions(-DPROJECT_NUMBER="${PROJECT_NUMBER}")

set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
message("PLATFORM :${PLATFORM}")
add_definitions(-DNODE_NAME="${PROJECT_NAME}")

if(${PLATFORM} STREQUAL "amd64")
  add_definitions(-DHJ_AMD64)
  add_definitions(-DCONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/${PROJECT_NUMBER}/${PLATFORM}/config.json")
  add_definitions(-DSO_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}")
  add_definitions(-DLOG_CONFIG_PATH="${CMAKE_SOURCE_DIR}/hj_manager/config/${PLATFORM}/hj_log.config")
  
elseif(${PLATFORM} STREQUAL "aarch64-linux-gnu")
  add_definitions(-DHJ_AARCH64)
  add_definitions(-DCONFIG_PATH="/home/robot/data/hj/config/config.json")
  add_definitions(-DSO_PATH="")
  add_definitions(-DLOG_CONFIG_PATH="/home/robot/data/hj/config/hj_log.config")
endif()
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_executable(${NODE_NAME} src/slam_node_manager.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} dl)
target_link_libraries(${NODE_NAME} base_framework)
add_custom_command(
  TARGET ${NODE_NAME}
  POST_BUILD
  COMMAND
  cp ${CMAKE_SOURCE_DIR}/hj_interface/platforms/${PLATFORM}/libcore_dump.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${NODE_NAME}
  COMMAND
  cp ${CMAKE_SOURCE_DIR}/hj_interface/platforms/${PLATFORM}/liblog_redirect.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${NODE_NAME}
  )
