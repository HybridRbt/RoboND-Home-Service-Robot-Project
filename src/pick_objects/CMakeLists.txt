cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS actionlib move_base_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pick_objects pick_objects.cpp)

target_link_libraries(pick_objects ${catkin_LIBRARIES})
