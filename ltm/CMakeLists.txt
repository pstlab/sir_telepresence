cmake_minimum_required(VERSION 3.0.2)
project(ltm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/include/json /usr/local/include/smt /usr/local/include/core /usr/local/include/riddle /usr/local/include/solver /usr/local/include/executor)
add_compile_definitions(BUILD_LISTENERS)

add_executable(ltm main.cpp local_task_manager.cpp)
target_link_libraries(ltm ${catkin_LIBRARIES} json smt core riddle solver executor)