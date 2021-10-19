cmake_minimum_required(VERSION 3.0.2)
project(deliberative_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch msgs)
roslaunch_add_file_check(launch)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/include/json /usr/local/include/smt /usr/local/include/core /usr/local/include/riddle /usr/local/include/solver /usr/local/include/executor)
add_compile_definitions(BUILD_LISTENERS)

add_executable(deliberative_tier main.cpp deliberative_manager.cpp deliberative_executor.cpp)
target_link_libraries(deliberative_tier ${catkin_LIBRARIES} json smt core riddle solver executor)
add_dependencies(deliberative_tier ${catkin_EXPORTED_TARGETS})