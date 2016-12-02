cmake_minimum_required(VERSION 2.8.3)
project(ekf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  angles
  cmake_modules
  roscpp
  sensor_msgs
  tf
  laser_geometry
  pcl_ros
)

find_package(Eigen REQUIRED)

find_package( PCL REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_geometry
  DEPENDS Boost Eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## Declare a C++ library
# add_library(ekf
#   src/${PROJECT_NAME}/ekf.cpp
# )


add_executable(ekf src/ekf.cpp)
add_dependencies(ekf nav_msgs_generate_messages_cpp)
target_link_libraries(ekf ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

