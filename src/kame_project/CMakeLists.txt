cmake_minimum_required(VERSION 2.8.3)
project(kame_project)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kobuki_msgs
  sensor_msgs
  rospy
  roscpp
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   kobuki_msgs
   std_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project1
  CATKIN_DEPENDS geometry_msgs kobuki_msgs rospy roscpp std_msgs message_runtime sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(explore src/explore.cpp)
target_link_libraries(explore ${catkin_LIBRARIES})
add_dependencies(explore project1_generate_messages_cpp)

