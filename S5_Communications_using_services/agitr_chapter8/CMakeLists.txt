# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(agitr_chapter8)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    turtlesim 
)

# Declare our catkin package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demopkg
   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(pubvel_toggle src/pubvel_toggle.cpp)
add_executable(spawn_turtle src/spawn_turtle.cpp)

# Specify libraries against which to link.
target_link_libraries(pubvel_toggle ${catkin_LIBRARIES})
target_link_libraries(spawn_turtle ${catkin_LIBRARIES})
