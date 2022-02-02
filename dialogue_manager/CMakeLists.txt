cmake_minimum_required(VERSION 3.0.2)
project(dialogue_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy deliberative_tier persistence_manager std_msgs message_generation)

## Declare ROS messages and services
add_message_files(
    FILES
    dialogue_state.msg
    button.msg
)

add_service_files(
    FILES
    utterance_to_pronounce.srv
    face_to_show.srv
    question_to_ask.srv
    utterance_to_recognize.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/google_stt.py scripts/google_tts.py scripts/dialogue.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})