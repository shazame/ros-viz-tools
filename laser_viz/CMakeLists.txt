cmake_minimum_required(VERSION 2.8.3)
project(laser_viz)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_viz
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

