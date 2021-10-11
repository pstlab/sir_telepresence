cmake_minimum_required(VERSION 3.0.2)
project(sequencer_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sequencer_tier main.cpp sequencer.cpp)
target_link_libraries(sequencer_tier ${catkin_LIBRARIES})