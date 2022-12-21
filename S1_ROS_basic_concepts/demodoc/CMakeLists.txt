# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(demodoc)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    turtlesim 
    std_msgs 
    message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Changerate.srv
 )
 
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

# Declare our catkin package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demopkg
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(pubvel_toggle_plus src/pubvel_toggle_plus.cpp) 

# Specify libraries against which to link.
target_link_libraries(pubvel_toggle_plus ${catkin_LIBRARIES})
add_dependencies(pubvel_toggle_plus demodoc_gencpp) 