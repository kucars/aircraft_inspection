cmake_minimum_required(VERSION 2.8.3)
project(aircraft_inspection)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-D__PX4_ROS)
add_definitions(-D__EXPORT=)
add_definitions(-DMAVLINK_DIALECT=common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules
  gazebo_msgs
  sensor_msgs
  mav_msgs
  libmavconn
  tf
)

find_package(Eigen REQUIRED)
catkin_package(
  INCLUDE_DIRS #src/include
  #LIBRARIES px4
  #CATKIN_DEPENDS message_runtime
  #roscpp rospy std_msgs libmavconn
  #DEPENDS system_lib
)





