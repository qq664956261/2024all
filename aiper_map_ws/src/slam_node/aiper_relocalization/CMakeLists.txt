cmake_minimum_required(VERSION 3.0.2)
project(aiper_relocalization)
set (CMAKE_CXX_STANDARD 14)
#  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
SET(CMAKE_BUILD_TYPE Release)
set( HJ_WORKSPACE_PATH "../..")
set(RELEASE_VER "debug" CACHE STRING "debug or release")
if(${RELEASE_VER} STREQUAL "debug")
elseif(${RELEASE_VER} STREQUAL "release")
  add_definitions(-DCONFIG_ENABLE_LOG=4)
endif()
set(PLATFORM "amd64" CACHE STRING "amd64, i386, aarch64-linux-gnu, etc.")
set(MODE "aiper_slam" CACHE STRING "aiper_relocalization, aiper_slam...")
set(INSTALL_DEVEL_PATH slam_node)
add_definitions(-DENABLE_CSTLOG)
add_definitions(-DRELOCALIZATION_DEBUG)
# add_definitions(-DX9)

find_package(catkin REQUIRED COMPONENTS rostime roscpp
  std_msgs
  geometry_msgs
  visualization_msgs)
# find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
        filesystem)

include(cmake/eigen.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories(
  "${HJ_WORKSPACE_PATH}/slam_node/aiper_relocalization/thirdparty/sophus"
  "${HJ_WORKSPACE_PATH}/slam_node/aiper_slam_common"
  # ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="aiper_relocalization")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

message("**********************MODE: ${MODE}**********************")
if(MODE STREQUAL "aiper_slam")
  add_library(aiper_relocalization src/relocalization.cpp
  src/kdtree.cpp
  src/icp_3d.cpp
  src/scancontext.cpp
  ${HJ_WORKSPACE_PATH}/slam_node/aiper_slam_common/common_data/encoder_data.cpp
  ${HJ_WORKSPACE_PATH}/slam_node/aiper_slam_common/common_data/mag_data.cpp
  ${HJ_WORKSPACE_PATH}/slam_node/aiper_slam_common/common_data/pressure_data.cpp
  )
else()
  add_library(aiper_relocalization src/relocalization.cpp
  src/kdtree.cpp
  src/icp_3d.cpp
  src/scancontext.cpp
  src/aiper_relocalization.cpp
  )
endif()
# target_link_libraries(aiper_relocalization ${catkin_LIBRARIES} dl ${OpenCV_LIBRARIES} hjlog ${Boost_LIBRARIES})
target_link_libraries(aiper_relocalization ${catkin_LIBRARIES} dl ${Boost_LIBRARIES})
target_link_libraries(aiper_relocalization base_framework)
set_target_properties(aiper_relocalization PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})
