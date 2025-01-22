cmake_minimum_required(VERSION 3.10)
project(aiper_slam_fusion)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-std=c++17")
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, aarch64-linux-gnu, etc.")
set(INSTALL_DEVEL_PATH slam_node)

#set(CMAKE_PREFIX_PATH "/opt/ros/noetic")
#set(CMAKE_MODULE_PATH "${CMAKE_PREFIX_PATH}/share/catkin/cmake")
set(${CATKIN_DEVEL_PREFIX} ${PROJECT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CATKIN_DEVEL_PREFIX}/share/catkin/cmake)

add_definitions(-DFUNCTION_NAME="aiper_slam_fusion")
add_definitions(-DDEMO_SHOW)
#add_definitions(-DWith_mag)
# add_definitions(-DX9)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 	
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86") 	
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
endif()

if (PLATFORM STREQUAL "aarch64-linux-gnu")
  message("*********Platform ${PLATFORM}************")
  set(G2O_DIR ${PROJECT_SOURCE_DIR}/lib/g2o)
  include_directories(${G2O_DIR}/include/)
  file(GLOB g2o_libs "${G2O_DIR}/lib/*.so")
else()
  message("*********Platform ${PLATFORM}************")
  add_definitions(-DPC_ENABLE)
  add_definitions(-DDEMO_SHOW)
  list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
  include(cmake/g2o.cmake)
endif()

include(cmake/catkin.cmake)
if (PLATFORM STREQUAL "aarch64-linux-gnu")
  #  include_directories("${PROJECT_SOURCE_DIR}/include/eigen3")
  #  include_directories("${PROJECT_SOURCE_DIR}/include/eigen3/Eigen")
  include(cmake/eigen.cmake)

else()
  include(cmake/opencv.cmake)
  include(cmake/eigen.cmake)
endif()

add_subdirectory(3rd/GeographicLib GeographicLib)
file(GLOB MAP_SOURCES "${PROJECT_SOURCE_DIR}/src/map_modules/*")
file(GLOB MAP_HEADERS "${PROJECT_SOURCE_DIR}/include/map_modules/")
include_directories(
  ${PROJECT_SOURCE_DIR}/3rd/GeographicLib/include/
  ${PROJECT_SOURCE_DIR}/include
  ${MAP_HEADERS}
)

include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("${HJ_WORKSPACE_PATH}/slam_node/aiper_relocalization/include")
include_directories("${HJ_WORKSPACE_PATH}/slam_node/aiper_relocalization/thirdparty/sophus")
include_directories("${HJ_WORKSPACE_PATH}/slam_node/aiper_slam_common")

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")

file(GLOB SLAM_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")
add_library(aiper_slam_fusion SHARED
  ${SLAM_SOURCES}
  ${MAP_SOURCES}
)


if(PLATFORM STREQUAL "aarch64-linux-gnu")
  target_link_libraries(aiper_slam_fusion ${opencvlibs} ${g2o_libs})
else()
  target_link_libraries(aiper_slam_fusion
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ${CSPARSE_LIBRARY})
endif()


target_link_libraries(aiper_slam_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(aiper_slam_fusion dl)
target_link_libraries(aiper_slam_fusion base_framework)
target_link_libraries(aiper_slam_fusion aiper_relocalization)

set_target_properties(aiper_slam_fusion PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})
