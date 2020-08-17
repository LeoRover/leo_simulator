cmake_minimum_required(VERSION 3.0.2)
project(leo_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_differential_plugin 
  src/differential_plugin.cpp)
target_link_libraries(gazebo_differential_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS scripts/odom_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS gazebo_differential_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)