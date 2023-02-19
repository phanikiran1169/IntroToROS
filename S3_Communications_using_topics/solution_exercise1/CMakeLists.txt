# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(solution_exercise1)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs turtlesim)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(pubvelsafe src/pubvelsafe.cpp)
add_executable(subpose src/subpose.cpp)

# Specify libraries against which to link.
target_link_libraries(pubvelsafe ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})

