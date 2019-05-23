cmake_minimum_required(VERSION 2.8.6)

project(task_planner_ros_wrapper)

########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  message_generation
  genmsg 
  actionlib_msgs 
  std_msgs
  diagnostic_msgs
  ropod_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY ros/msg
  FILES
  TaskPlannerPredicate.msg
  TaskPlannerFluent.msg
)

add_service_files(
  DIRECTORY ros/srv
  FILES
  TaskPlannerUpdateKB.srv
  TaskPlannerQueryKB.srv
)

add_action_files(
  DIRECTORY ros/action
  FILES
  TaskPlannerPlan.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  diagnostic_msgs
  ropod_ros_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs diagnostic_msgs ropod_ros_msgs
)

