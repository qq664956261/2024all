cmake_minimum_required(VERSION 2.8.3)
project(slam_core)

# set path for library file output
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_CXX_STANDARD 17)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)


include_directories(
    include
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    include
    ${PCL_LIBRARY_DIRS}
)

add_library(slam_core SHARED
    src/Localizer.cpp
    src/Mapper.cpp
)

target_link_libraries(slam_core
	PUBLIC ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs yaml-cpp
)
