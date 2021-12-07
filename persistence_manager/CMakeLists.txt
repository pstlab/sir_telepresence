cmake_minimum_required(VERSION 3.0.2)
project(persistence_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

## Declare ROS messages and services
add_service_files(
    FILES
    get_state.srv
    set_state.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/storage.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})