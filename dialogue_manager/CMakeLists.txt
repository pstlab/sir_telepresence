cmake_minimum_required(VERSION 3.0.2)
project(dialogue_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy deliberative_tier persistence_manager roslaunch std_msgs message_generation)
roslaunch_add_file_check(launch)

## Declare ROS messages and services
add_message_files(
    FILES
    dialogue_state.msg
)

add_service_files(
    FILES
    get_string.srv
    set_string.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/google_stt.py scripts/google_tts.py scripts/dialogue.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})