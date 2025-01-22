# CMakeLists.txt for EuRoC image reader project

cmake_minimum_required(VERSION 3.10)
project(EuRoCImageReader)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# Find OpenCV package
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
include_directories(
${OpenCV_INCLUDE_DIRS}
${OpenGL_INCLUDE_DIRS}
imgui)

# Add executable
add_executable(read_euroc_images read_euroc_images.cpp gui.cpp)

# Link OpenCV libraries
target_link_libraries(read_euroc_images ${OpenCV_LIBS} OpenGL::GL  GLEW::GLEW glfw)
