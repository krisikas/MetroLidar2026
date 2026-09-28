cmake_minimum_required(VERSION 3.16)
project(metro_obstacle_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(metro_obstacle_detector_interfaces REQUIRED)

# OpenMP for multi-core parallelism
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  add_compile_definitions(HAVE_OPENMP=1)
endif()

# Python link for macOS RoboStack compatibility
if(APPLE)
  find_package(Python3 COMPONENTS Development)
endif()

include_directories(
  include
)

# Core pipeline library
set(DETECTOR_SOURCES
  src/cloud_parser.cpp
  src/dual_rail_tracker.cpp
  src/track_geometry_spline.cpp
  src/clearance_envelope_sdf.cpp
  src/specialized_detectors.cpp
  src/temporal_obstacle_tracker.cpp
  src/train_velocity_estimator.cpp
  src/decision_engine.cpp
  src/obstacle_detector_pipeline.cpp
)

add_library(${PROJECT_NAME}_core SHARED
  ${DETECTOR_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  metro_obstacle_detector_interfaces
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)
endif()

if(APPLE AND Python3_Development_FOUND)
  target_link_libraries(${PROJECT_NAME}_core Python3::Python)
endif()

# Node executable
add_executable(${PROJECT_NAME}_node
  src/obstacle_detector_node.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  metro_obstacle_detector_interfaces
)

if(APPLE AND Python3_Development_FOUND)
  target_link_libraries(${PROJECT_NAME}_node Python3::Python)
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_clearance_sdf
    test/test_clearance_sdf.cpp
  )
  target_link_libraries(test_clearance_sdf
    ${PROJECT_NAME}_core
  )
  if(APPLE AND Python3_Development_FOUND)
    target_link_libraries(test_clearance_sdf Python3::Python)
  endif()

  ament_add_gtest(test_specialized_detectors
    test/test_specialized_detectors.cpp
  )
  target_link_libraries(test_specialized_detectors
    ${PROJECT_NAME}_core
  )
  if(APPLE AND Python3_Development_FOUND)
    target_link_libraries(test_specialized_detectors Python3::Python)
  endif()
endif()

# Installation
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
  metro_obstacle_detector_interfaces
)

ament_package()
