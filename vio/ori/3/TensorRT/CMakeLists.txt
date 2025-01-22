cmake_minimum_required(VERSION 3.10)
project(learned_features_inference_tensorrt)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

find_package(OpenCV REQUIRED)
set(TensorRT_DIR "/home/zc/lib/TensorRT-10.2.0.19.Linux.x86_64-gnu.cuda-12.5/TensorRT-10.2.0.19")

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} inc ${TensorRT_DIR}/include)
link_directories(${TensorRT_DIR}/lib)

add_executable(tensorrt_demo main.cpp src/interface.cpp src/extractor.cpp src/utils.cpp)

target_link_libraries(tensorrt_demo ${OpenCV_LIBS} nvinfer nvinfer_plugin ${CUDA_LIBRARIES})


