cmake_minimum_required(VERSION 3.1.8)
cmake_policy(SET CMP0057 NEW)

project(cohan_sim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FindPkgConfig)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")

  # ROS 2 setup
  add_definitions(-DROS=2)
  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(X11 REQUIRED)

  pkg_check_modules(SDL2 REQUIRED sdl2)

  include_directories(
    ${SDL2_INCLUDE_DIRS}
    include
  )

  # Add sim.cpp as a library
  add_library(libsim SHARED src/sim.cpp)
  target_link_libraries(libsim ${SDL2_LIBRARIES} yaml-cpp X11)

  add_executable(simros_node src/simros.cpp)
  ament_target_dependencies(simros_node rclpy nav_msgs geometry_msgs sensor_msgs tf2_ros)
  target_link_libraries(simros_node libsim ${SDL2_LIBRARIES} yaml-cpp X11)

  # Install executable
  install(TARGETS simros_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install library
  install(TARGETS libsim
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # Install include directory
  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()

else()

  ## Check the version of ubuntu to support Simulators
  execute_process(
    COMMAND lsb_release -rs
    OUTPUT_VARIABLE UBUNTU_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(UBUNTU_VERSION MATCHES "^18")
    add_definitions(-DUBUNTU_18)
  endif()

  # ROS 1 setup
  add_definitions(-DROS=1)
  find_package(catkin REQUIRED COMPONENTS
    rospy
    nav_msgs
    geometry_msgs
    sensor_msgs
    tf2_ros
  )
  find_package(X11 REQUIRED)
  pkg_check_modules(SDL2 REQUIRED sdl2)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  cohan_sim
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    include
  )

  # Add sim.cpp as a library (uses simros node)
  add_library(libsim SHARED src/sim.cpp)
  target_link_libraries(libsim ${SDL2_LIBRARIES} yaml-cpp X11) 

  ## Add the executable for rosnode wrapper
  add_executable(simros_node src/simros.cpp)
  target_link_libraries(simros_node ${catkin_LIBRARIES} ${SDL2_LIBRARIES} libsim)

endif()
