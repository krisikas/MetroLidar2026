cmake_minimum_required(VERSION 3.12)
project(metro_tunnel_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Development QUIET)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
)

# Core algorithmic and component library
add_library(metro_tunnel_tracker_core SHARED
  src/rail_geometry_tracker.cpp
  src/tunnel_tracker_node.cpp
)

target_include_directories(metro_tunnel_tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(metro_tunnel_tracker_core ${DEPENDENCIES})

if(Python3_FOUND AND APPLE)
  target_link_libraries(metro_tunnel_tracker_core Python3::Python)
endif()

# Register as ROS 2 component
rclcpp_components_register_node(
  metro_tunnel_tracker_core
  PLUGIN "metro_tunnel_tracker::TunnelTrackerNode"
  EXECUTABLE tunnel_tracker_component_node
)

# Standalone executable
add_executable(tunnel_tracker_node
  src/main.cpp
)

target_link_libraries(tunnel_tracker_node
  metro_tunnel_tracker_core
)

if(Python3_FOUND AND APPLE)
  target_link_libraries(tunnel_tracker_node Python3::Python)
endif()

ament_target_dependencies(tunnel_tracker_node ${DEPENDENCIES})

# Automated Unit Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry_tracker
    test/test_geometry_tracker.cpp
  )
  target_link_libraries(test_geometry_tracker
    metro_tunnel_tracker_core
  )
  if(Python3_FOUND AND APPLE)
    target_link_libraries(test_geometry_tracker Python3::Python)
  endif()
endif()

# Installation rules
install(TARGETS
  metro_tunnel_tracker_core
  tunnel_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

