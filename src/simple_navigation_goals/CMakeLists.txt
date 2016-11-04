cmake_minimum_required(VERSION 2.8.3)
project(simple_navigation_goals)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_link_navigation_client src/base_link_navigation_client.cpp)
target_link_libraries(base_link_navigation_client ${catkin_LIBRARIES})
add_dependencies(base_link_navigation_client ${catkin_EXPORTED_TARGETS})

add_executable(map_navigation_client src/map_navigation_client.cpp)
target_link_libraries(map_navigation_client ${catkin_LIBRARIES})
add_dependencies(map_navigation_client ${catkin_EXPORTED_TARGETS})
