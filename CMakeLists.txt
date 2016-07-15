cmake_minimum_required(VERSION 2.8.3)
project(labrom_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/PID_simple.cfg
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
    CATKIN_DEPENDS dynamic_reconfigure
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
  ${Boost_INCLUDE_DIRS}
  include
)
 
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pid_simple.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)