cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  dwa_planner_core
  dwa_planner_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # dwa_planner
  CATKIN_DEPENDS 
    roscpp
    grid_map_core
    grid_map_ros
    dwa_planner_core
    dwa_planner_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(dwa_planner_node src/dwa_planner_node.cpp
  src/DwaPlanner.cpp
)
target_link_libraries(dwa_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)