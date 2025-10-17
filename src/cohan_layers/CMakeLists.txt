cmake_minimum_required(VERSION 3.5)
project(cohan_layers)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(agent_path_prediction REQUIRED)
find_package(ros2_helpers REQUIRED)


# Note: dynamic_reconfigure is not available in ROS 2
# Use node parameters with declare_parameter and parameter callbacks instead

include_directories(
  include
)

add_library(cohan_layers SHARED
  src/agent_layer.cpp
  src/static_agent_layer.cpp
  src/agent_visibility_layer.cpp
)

ament_target_dependencies(cohan_layers
  rclcpp
  angles
  nav2_costmap_2d
  geometry_msgs
  cohan_msgs
  pluginlib
  agent_path_prediction
  ros2_helpers
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS cohan_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(cohan_layers)
ament_export_dependencies(
  rclcpp
  angles
  nav2_costmap_2d
  geometry_msgs
  cohan_msgs
  pluginlib
  agent_path_prediction
  ros2_helpers
)

ament_package()
