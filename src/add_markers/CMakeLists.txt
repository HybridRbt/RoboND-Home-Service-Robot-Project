cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)
catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(add_markers add_markers.cpp)

target_link_libraries(add_markers ${catkin_LIBRARIES})
