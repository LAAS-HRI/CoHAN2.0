cmake_minimum_required(VERSION 3.1.0)
project(hateb_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) # None, Debug, Release, RelWithDebInfo, MinSizeRel

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  costmap_converter
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  message_generation
  nav_core
  nav_msgs
  mbf_costmap_core
  mbf_msgs
  roscpp
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
)
message(STATUS "System: ${CMAKE_SYSTEM}")

# # System dependencies are found with CMake's conventions
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

# message(STATUS ${G2O_LIBRARIES})

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# # C++11 support
# # Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
# # Even if the ROS Jade specifications do not want c++11-only packages,
# # we cannot compile without c++11 enabled. Another option would be to downgrade
# # libg2o third-party package.
# # By now, if you do not want c++11, please refer to the ros indigo version.
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

  if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support which is required
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
  endif()
endif()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependencies might have been
# #     pulled in transitively but can be declared for certainty nonetheless:
# #     * add a build_depend tag for "message_generation"
# #     * add a run_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObstacleMsg.msg
  OptimizationCost.msg
  OptimizationCostArray.msg
  FeedbackMsg.msg
  PlanningMode.msg
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs costmap_converter cohan_msgs
)

# add dynamic reconfigure api
# find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/HATebLocalPlannerReconfigure.cfg
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if you package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS} ../3rdparty/g2o/include
  LIBRARIES hateb_local_planner ${EXTERNAL_LIBS}
  CATKIN_DEPENDS
  base_local_planner
  costmap_2d
  costmap_converter
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  message_runtime
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  tf2
  tf2_ros
  cohan_msgs
  visualization_msgs
  behaviortree_cpp
  agent_path_prediction
  DEPENDS SUITESPARSE G2O
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# # Build the hateb_local_planner library
add_library(hateb_local_planner
  src/timed_elastic_band.cpp
  src/optimal_planner.cpp
  src/obstacles.cpp
  src/visualization.cpp
  src/recovery_behaviors.cpp
  src/hateb_config.cpp
  src/hateb_local_planner_ros.cpp
  src/backoff.cpp
  src/behavior_tree/condition/is_goal_reached.cpp
  src/behavior_tree/condition/is_goal_updated.cpp
  src/behavior_tree/condition/vel_obs_exit_condition.cpp
  src/behavior_tree/condition/backoff_exit_condition.cpp
  src/behavior_tree/condition/single_band_exit_condition.cpp
  src/behavior_tree/condition/dual_band_exit_condition.cpp
  src/behavior_tree/condition/passthrough_condition.cpp
  src/behavior_tree/action/set_mode.cpp
  src/mode_switch.cpp
)

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(hateb_local_planner ${PROJECT_NAME}_gencfg)

# Generate messages before compiling the lib
add_dependencies(hateb_local_planner ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(hateb_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hateb_local_planner
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/ros1_ros2_bridge.py
  scripts/static_plan_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark executables and/or libraries for installation
install(TARGETS hateb_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# # Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}

  # FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# # Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  hateb_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch cfg scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# ############
# # Testing ##
# ############
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(test_teb_basics test/teb_basics.cpp)

#   if(TARGET test_teb_basics)
#     target_link_libraries(test_teb_basics hateb_local_planner)
#   endif()
# endif()

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hateb_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
