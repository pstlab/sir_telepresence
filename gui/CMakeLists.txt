cmake_minimum_required(VERSION 3.0.2)
project(gui)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy msgs)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/gui.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})