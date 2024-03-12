cmake_minimum_required(VERSION 3.0.2)
project(pioneer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
  std_msgs
  urdf
  xacro
)


find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
  roscpp
  rospy
  gazebo_ros
  gazebo_ros_control
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(
  ${PROJECT_NAME} 
  src/pioneer_plugin.cc
  src/Walkingpattern_generator.cc
  src/test.cc
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${Boost_LIBRARIES}
)

