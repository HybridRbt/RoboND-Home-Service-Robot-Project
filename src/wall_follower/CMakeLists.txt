cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wall_follower wall_follower.cpp)

target_link_libraries(wall_follower ${catkin_LIBRARIES})
