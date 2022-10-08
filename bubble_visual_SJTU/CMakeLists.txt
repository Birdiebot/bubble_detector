cmake_minimum_required(VERSION 3.5)
project(bubble_visual_sjtu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(CUDA)
find_library(NVINFER NAMES nvinfer)
find_library(NVINFERPLUGIN NAMES nvinfer_plugin)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSER NAMES nvonnxparser)
find_library(NVONNXPARSERRUNTIME NAMES nvonnxparser_runtime)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  bboxes_ex_msgs
)

set(SOURCES
  src/cv_base_node.cpp
  src/TRTModule.cpp
  /usr/src/tensorrt/samples/common/logger.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
  include/cv_base_node.hpp
  include/TRTModule.hpp
  /usr/src/tensorrt/samples/common/logger.h
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  nvinfer
  nvinfer_plugin
  nvparsers
  nvonnxparser
  # nvonnxparser_runtime
  cuda
)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
  CUDA
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "VisualNode"
  EXECUTABLE Visual
)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
