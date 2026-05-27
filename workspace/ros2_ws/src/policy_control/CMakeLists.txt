cmake_minimum_required(VERSION 3.8)
project(policy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS
    /opt/onnxruntime/include
    $ENV{ONNXRUNTIME_ROOT}/include
)

find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS
    /opt/onnxruntime/lib
    $ENV{ONNXRUNTIME_ROOT}/lib
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime C++ headers/library not found. Rebuild the Docker image first.")
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(policy_controller_cpp
  src/policy_controller_cpp.cpp
)

target_include_directories(policy_controller_cpp PRIVATE
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

ament_target_dependencies(policy_controller_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(policy_controller_cpp
  ${ONNXRUNTIME_LIBRARY}
)

install(TARGETS
  policy_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/observation_publisher
  scripts/policy_inference
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
