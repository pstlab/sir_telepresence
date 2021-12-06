cmake_minimum_required(VERSION 3.0.2)
project(msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

## Declare ROS messages and services
add_message_files(
    FILES
    navigation_state.msg
)

add_service_files(
    FILES
    set_string.srv
    get_string.srv
    get_state.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()