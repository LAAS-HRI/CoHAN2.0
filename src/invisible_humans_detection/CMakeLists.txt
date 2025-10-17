cmake_minimum_required(VERSION 3.5)
project(invisible_humans_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_helpers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  ros2_helpers
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  cohan_msgs
  costmap_converter_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/invisible_humans_detection.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/invisible_humans_detection.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
