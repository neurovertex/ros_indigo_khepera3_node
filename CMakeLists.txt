cmake_minimum_required(VERSION 2.8.3)
project(khepera_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PLAYERCPP REQUIRED playerc++)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES khepera_node
#  CATKIN_DEPENDS roscpp tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PLAYERCPP_INCLUDE_DIRS}
)

LINK_DIRECTORIES(${PLAYERCPP_LIBRARY_DIRS})

add_executable(khepera_node src/khepera_node.cpp)

TARGET_LINK_LIBRARIES(khepera_node ${catkin_LIBRARIES} ${PLAYERCPP_LIBRARIES})

