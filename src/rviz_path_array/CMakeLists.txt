cmake_minimum_required(VERSION 3.5)
project(rviz_path_array)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(cohan_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/patharray_display.cpp
)

set(HEADER_FILES
  src/patharray_display.hpp
)

# Create the library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rviz_rendering
  rviz_default_plugins
  cohan_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rviz_common
  rviz_rendering
  rviz_default_plugins
  cohan_msgs
  pluginlib
)

ament_package()
