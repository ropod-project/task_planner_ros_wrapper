cmake_minimum_required(VERSION 2.8.6)

project(task_planner_ros_wrapper)

########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg 
  actionlib_msgs 
  actionlib
  std_msgs
  rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY ros/msg
  FILES
  KnowledgeItem.msg
)

add_service_files(
  DIRECTORY ros/srv
  FILES
  KnowledgeUpdateService.srv
  KnowledgeQueryService.srv
)

# add_action_files(
#   DIRECTORY ros/action
#   FILES
# )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs
)

