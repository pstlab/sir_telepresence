cmake_minimum_required(VERSION 3.0.2)
project(msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(
    FILES
    can_start.srv
    start_task.srv
    end_task.srv
    task_finished.srv
    create_reasoner.srv
    new_requirement.srv
    notify_reasoner_state.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()