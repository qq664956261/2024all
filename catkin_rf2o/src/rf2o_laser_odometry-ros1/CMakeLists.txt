cmake_minimum_required(VERSION 3.1)
project(rf2o_laser_odometry)

# 使用C++14标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS  
  cmake_modules
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  laser_geometry
)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS cmake_modules nav_msgs roscpp sensor_msgs std_msgs tf
 DEPENDS Eigen
)

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/CLaserOdometry2D.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(rf2o_laser_odometry_node src/CLaserOdometry2DNode.cpp)
target_link_libraries(rf2o_laser_odometry_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## 安装目标
install(TARGETS ${PROJECT_NAME} rf2o_laser_odometry_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)