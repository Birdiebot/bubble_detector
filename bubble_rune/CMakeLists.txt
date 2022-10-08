cmake_minimum_required(VERSION 3.5)
project(bubble_rune)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)        
find_package(std_msgs REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)

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
  # src/filter.cpp
  # src/detector.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_library(${PROJECT_NAME} SHARED
    include/cv_base_node.hpp
    include/filter.hpp
    include/detector.hpp
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )

  
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "VisualRune"
  EXECUTABLE VisualRune
)


# Install binaries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# Ament exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()