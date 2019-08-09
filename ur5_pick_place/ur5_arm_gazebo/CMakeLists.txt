cmake_minimum_required(VERSION 2.8.3)

project(ur5_arm_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
