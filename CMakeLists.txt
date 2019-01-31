cmake_minimum_required(VERSION 2.8.3)
project(base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  wheel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(LIB_PATH "./lib/")
FILE(GLOB_RECURSE LIB_SRC 
  "${LIB_PATH}/src/*.cpp"
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  include
  lib/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_state src/robot_state.cpp ${LIB_SRC})
target_link_libraries(robot_state ${catkin_LIBRARIES})
