cmake_minimum_required(VERSION 3.0.2)
project(aiper_map)
set (CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE Release)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, aarch64-linux-gnu, etc.")
set(INSTALL_DEVEL_PATH slam_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp
  std_msgs
  visualization_msgs)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include(cmake/eigen.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("${HJ_WORKSPACE_PATH}/slam_project/aiper_map/thirdparty/sophus"
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="aiper_map")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(aiper_map src/Relocalization.cpp
src/kdtree.cpp
src/icp_3d.cpp
src/Scancontext.cpp
src/aiper_map.cpp
)
target_link_libraries(aiper_map ${catkin_LIBRARIES} dl ${OpenCV_LIBRARIES} glog)
target_link_libraries(aiper_map base_framework)
set_target_properties(aiper_map PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})
