cmake_minimum_required(VERSION 2.8.3)

project(ur5_arm_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation 
  std_msgs
)

add_service_files(
  FILES
  SpawnTarget.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)