cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs deliberative_tier dialogue_manager)
find_package(Crow)

## Declare a catkin package
catkin_package()

add_executable(${PROJECT_NAME} main.cpp gui_server.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Crow::Crow)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})