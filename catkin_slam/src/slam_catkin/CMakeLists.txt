cmake_minimum_required(VERSION 2.8.3)
project(slam_catkin)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  roscpp
  rosbag_storage
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag_storage geometry_msgs nav_msgs roscpp std_msgs
  DEPENDS PCL EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ../slam_core/include
    ../slam_core/src
)

link_directories(
    include
    ${PCL_LIBRARY_DIRS}
    ../slam_core/lib
)
set(THIRD_PARTY_LIBS
  ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}  stdc++fs pthread yaml-cpp)

add_library(slam_catkin
  src/slam_catkin.cpp)
#target_link_libraries(slam_catkin slam_core ${THIRD_PARTY_LIBS})
target_link_libraries(slam_catkin  ${THIRD_PARTY_LIBS})

add_executable(mapper main_mapper.cpp)
add_dependencies(mapper ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mapper slam_core slam_catkin ${THIRD_PARTY_LIBS})
target_link_libraries(mapper  slam_catkin ${THIRD_PARTY_LIBS})

add_executable(localizer main_localizer.cpp)
add_dependencies(localizer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(localizer slam_core slam_catkin ${THIRD_PARTY_LIBS})
target_link_libraries(localizer slam_catkin ${THIRD_PARTY_LIBS})

