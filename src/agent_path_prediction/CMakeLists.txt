
cmake_minimum_required(VERSION 3.8)
project(agent_path_prediction)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_helpers REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictedPoses.msg"
  "msg/PredictedGoal.msg"
  "msg/PredictedGoals.msg"
  "msg/AgentPose.msg"
  "msg/HumanInfo.msg"
  "msg/AgentsInfo.msg"
  "srv/AgentPosePredict.srv"
  "srv/AgentGoal.srv"
  DEPENDENCIES
  geometry_msgs
  cohan_msgs
)

# Include directories
include_directories(include)

# Create executable
add_executable(agent_path_predict
  src/agent_path_prediction.cpp
  src/predict_goal_ros.cpp
)

# Dependencies for executable
ament_target_dependencies(agent_path_predict
  rclcpp
  geometry_msgs
  cohan_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_costmap_2d
  visualization_msgs
  ros2_helpers
)

# Link yaml-cpp
target_link_libraries(agent_path_predict yaml-cpp)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(agent_path_predict "${cpp_typesupport_target}")

# Install executable
install(TARGETS agent_path_predict
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  cohan_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_costmap_2d
  visualization_msgs
  ros2_helpers
  rosidl_default_runtime
)

ament_package()