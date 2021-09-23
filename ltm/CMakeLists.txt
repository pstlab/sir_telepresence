cmake_minimum_required(VERSION 3.0.2)
project(ltm)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ltm main.cpp)
target_link_libraries(ltm ${catkin_LIBRARIES} executor)