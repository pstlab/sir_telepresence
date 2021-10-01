cmake_minimum_required(VERSION 3.0.2)
project(msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES speak.srv start_dialogue.srv dialogue_finished.srv task_finished.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()