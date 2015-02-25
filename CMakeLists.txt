cmake_minimum_required(VERSION 2.8.3)
project(python_mar_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  arm5e_arm_services
  pcl_manipulation
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf arm5e_arm_services pcl_manipulation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

