cmake_minimum_required(VERSION 3.5)
project(hateb_local_planner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) # None, Debug, Release, RelWithDebInfo, MinSizeRel

# Option to enable/disable Behavior Tree debug logging
option(ENABLE_BT_DEBUG "Enable Behavior Tree debug logging" OFF)

if(ENABLE_BT_DEBUG)
  add_compile_definitions(BTPRINT=1)
  message(STATUS "Behavior Tree debug logging is ENABLED")
else()
  add_compile_definitions(BTPRINT=0)
  message(STATUS "Behavior Tree debug logging is DISABLED")
endif()

# Find ament packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(agent_path_prediction REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_helpers REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")

# System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if(EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif(EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif(EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif(EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleMsg.msg"
  "msg/OptimizationCost.msg"
  "msg/OptimizationCostArray.msg"
  "msg/FeedbackMsg.msg"
  "msg/PlanningMode.msg"
  DEPENDENCIES geometry_msgs std_msgs costmap_converter_msgs cohan_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Specify additional locations of header files
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
)

# Set dependencies for ament_target_dependencies
set(dependencies
  ament_index_cpp
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  costmap_converter
  costmap_converter_msgs
  geometry_msgs
  interactive_markers
  nav_msgs
  std_msgs
  std_srvs
  pluginlib
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  agent_path_prediction
  cohan_msgs
  visualization_msgs
  behaviortree_cpp
  ros2_helpers
)

# Build the hateb_local_planner library
add_library(hateb_local_planner_core SHARED
  src/timed_elastic_band.cpp
  src/optimal_planner.cpp
  src/obstacles.cpp
  src/visualization.cpp
  src/recovery_behaviors.cpp
  src/hateb_config.cpp
  src/hateb_local_planner_ros.cpp
  src/backoff.cpp
  src/behavior_tree/condition/is_goal_updated.cpp
  src/behavior_tree/condition/vel_obs_exit_condition.cpp
  src/behavior_tree/condition/backoff_exit_condition.cpp
  src/behavior_tree/condition/single_band_exit_condition.cpp
  src/behavior_tree/condition/dual_band_exit_condition.cpp
  src/behavior_tree/condition/passthrough_condition.cpp
  src/behavior_tree/action/set_mode.cpp
  src/mode_switch.cpp
  src/agents_class.cpp
)



ament_target_dependencies(hateb_local_planner_core
  ${dependencies}
  agent_path_prediction
)

target_link_libraries(hateb_local_planner_core 
  ${EXTERNAL_LIBS}
  "${cpp_typesupport_target}"
)

# Install
install(TARGETS hateb_local_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  hateb_local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch cfg scripts behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS
  scripts/static_plan_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core hateb_local_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(hateb_local_planner_core)
ament_export_dependencies(${dependencies})

ament_package()
