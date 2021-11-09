cmake_minimum_required(VERSION 3.0.2)
project(deliberative_tier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GenerateExportHeader)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp roslib msgs)

## Declare a catkin package
catkin_package()

add_subdirectory(oRatio/smt)
add_subdirectory(oRatio/smt/json)
add_subdirectory(oRatio/riddle)
add_subdirectory(oRatio/core)
add_subdirectory(oRatio/solver)
add_subdirectory(oRatio/executor)
target_compile_definitions(solver PUBLIC BUILD_EXECUTOR)
target_compile_definitions(core PUBLIC BUILD_LISTENERS)
target_compile_definitions(solver PUBLIC BUILD_LISTENERS)

add_executable(deliberative_tier main.cpp deliberative_manager.cpp deliberative_executor.cpp)
target_include_directories(deliberative_tier PRIVATE include ${catkin_INCLUDE_DIRS} solver executor)
target_link_libraries(deliberative_tier ${catkin_LIBRARIES} json smt core riddle solver executor)
add_dependencies(deliberative_tier ${catkin_EXPORTED_TARGETS})