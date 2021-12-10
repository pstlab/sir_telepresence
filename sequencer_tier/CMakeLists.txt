cmake_minimum_required(VERSION 3.0.2)
project(sequencer_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp deliberative_tier dialogue_manager persistence_manager roslib roslaunch std_msgs message_generation)
roslaunch_add_file_check(launch)

## Declare ROS messages and services
add_message_files(
    FILES
    sequencer_state.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

add_executable(${PROJECT_NAME} main.cpp sequencer.cpp service_call.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})