cmake_minimum_required(VERSION 2.8.3)
project(euclid_rover5_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS system)



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs  
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(rover5_node src/rover5_node.cpp)

target_link_libraries(rover5_node
   ${catkin_LIBRARIES}
 )

