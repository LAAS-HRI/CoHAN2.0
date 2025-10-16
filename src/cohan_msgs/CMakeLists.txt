cmake_minimum_required(VERSION 3.16.3)
project(cohan_msgs)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_COMPILER_ARG1 -std=c++17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentPath.msg"
  "msg/AgentPathArray.msg"
  "msg/AgentTimeToGoalArray.msg"
  "msg/AgentTrajectory.msg"
  "msg/AgentTrajectoryArray.msg"
  "msg/AgentType.msg"
  "msg/TrackedSegment.msg"
  "msg/TrackedAgent.msg"
  "msg/TrackedAgents.msg"
  "msg/TrackedSegmentType.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryStamped.msg"
  "msg/TrajectoryArray.msg"
  "msg/StateArray.msg"
  "msg/AgentMarker.msg"
  "msg/AgentMarkerStamped.msg"
  "msg/PointArray.msg"
  "msg/PassageType.msg"
  "msg/CrossingInfo.msg"
  "srv/Optimize.srv"
  "srv/GetTrajectory.srv"
  "msg/AgentTimeToGoal.msg"
  "msg/TrajectoryPoint.msg"
  DEPENDENCIES geometry_msgs std_msgs nav_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
