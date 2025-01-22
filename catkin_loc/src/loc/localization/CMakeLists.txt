cmake_minimum_required(VERSION 2.8.3)
project(localization)
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
#SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  cf_msgs
  isas_msgs
)
set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/opencv.cmake)
include(cmake/ceres.cmake)
#include(cmake/eigen.cmake)
#include(cmake/geographic.cmake)
# sophus
include_directories(${PROJECT_SOURCE_DIR}/third_party/sophus)

include_directories(include ${catkin_INCLUDE_DIRS}
" usr/local/include/eigen3/")
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(test_eskf_imu_uwb_node src/test_eskf_imu_uwb_node.cpp src/imu_init/static_imu_init.cpp src/pre_integrator/parameters.cpp ${ALL_SRCS})
target_link_libraries(test_eskf_imu_uwb_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(test_graph_based_optimization_node src/test_graph_based_optimization_node.cpp src/imu_init/static_imu_init.cpp ${ALL_SRCS})
target_link_libraries(test_graph_based_optimization_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})