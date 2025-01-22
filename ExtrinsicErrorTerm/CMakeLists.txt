cmake_minimum_required(VERSION 3.1)

# 指定库名和版本号
project(ExtrinsicErrorTerm
  VERSION 0.1
  )

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
#find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package( Ceres REQUIRED )

find_package(g2o REQUIRED)

# 头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/sophus
  ${g2o_INCLUDE_DIRS}
  )




 add_executable("mapping"
 test_Mapping.cpp
 src/Mapping.cpp
 src/kdtree.cpp
 src/icp_3d.cpp
)
target_link_libraries("mapping"
#${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
"/usr/local/lib/libg2o_core.so"
g2o_core g2o_stuff g2o_types_slam3d
)

add_executable("relocalization"
test_Relocalization.cpp
src/Relocalization.cpp
src/kdtree.cpp
src/icp_3d.cpp
src/Scancontext.cpp
)
target_link_libraries("relocalization"
#${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
#${Boost_LIBRARIES}
glog

)





