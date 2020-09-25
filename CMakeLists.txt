cmake_minimum_required(VERSION 3.0.2)
project(leo_gazebo)

# Set CMP0054
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
)
find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES
    gazebo_differential_plugin
  CATKIN_DEPENDS 
    roscpp
    gazebo_ros
  DEPENDS
    gazebo
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(
  gazebo_differential_plugin 
  src/differential_plugin.cpp
)
target_link_libraries(gazebo_differential_plugin ${GAZEBO_LIBRARIES})

foreach(dir launch config models worlds)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)

catkin_install_python(
  PROGRAMS scripts/odom_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS gazebo_differential_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)