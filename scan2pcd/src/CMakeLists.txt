cmake_minimum_required(VERSION 3.1)

# 指定库名和版本号
project(scan2pcd)

# 使用现代 C++（C++ 11及以上）
set(CMAKE_CXX_STANDARD 17)
if(UNIX)
  add_compile_options(-fPIC)
endif()

#boost
find_package(Boost REQUIRED COMPONENTS
        timer
        filesystem)
#PCL
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package( Ceres REQUIRED )

find_package( Sophus REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)
catkin_package()
# 头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 
  )



add_executable(${PROJECT_NAME}
  main.cpp
 )
 add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${catkin_LIBRARIES}
)




