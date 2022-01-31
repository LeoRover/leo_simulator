cmake_minimum_required(VERSION 3.0.2)
project(leo_gazebo_plugins)

# Set CMP0054
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED 
  COMPONENTS
    gazebo_dev
    roscpp
)

catkin_package(
  LIBRARIES
    leo_gazebo_differential_plugin
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(leo_gazebo_differential_plugin 
  src/differential_plugin.cpp
)
target_link_libraries(leo_gazebo_differential_plugin
  ${catkin_LIBRARIES}
)

install(
  TARGETS leo_gazebo_differential_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
