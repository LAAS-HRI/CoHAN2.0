cmake_minimum_required(VERSION 2.8.3)
project(cohan_stage_navigation)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install
foreach(dir maps maps launch config rviz scripts)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
