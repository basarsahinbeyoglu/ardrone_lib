cmake_minimum_required(VERSION 3.0.2)
project(ardrone_lib)

find_package(catkin REQUIRED COMPONENTS
rospy
std_msgs
geometry_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

