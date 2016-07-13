cmake_minimum_required(VERSION 2.8.3)
project(labrom_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pid_simple.cpp
)
