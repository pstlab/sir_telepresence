cmake_minimum_required(VERSION 3.0.2)
project(home_automation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

## Declare ROS messages and services
add_message_files(
    FILES
    home_automation_state.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/home.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})