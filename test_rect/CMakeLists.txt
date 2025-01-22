cmake_minimum_required(VERSION 3.0.2)
project(test_rect)
set (CMAKE_CXX_STANDARD 14)
#  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
SET(CMAKE_BUILD_TYPE Release)



find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
        filesystem)


include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(test_rect test_rect.cpp)
target_link_libraries(test_rect ${Boost_LIBRARIES})
