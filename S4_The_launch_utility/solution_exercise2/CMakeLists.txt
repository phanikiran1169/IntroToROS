# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(solution_exercise2)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})