cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  ros_transform_utils
  ros_pcl_utils
  grid_map_core
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # dwa_planner
  CATKIN_DEPENDS 
    roscpp
    ros_node_utils
    ros_transform_utils
    ros_pcl_utils
    grid_map_core
    grid_map_ros
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

add_executable(dwa_planner_node src/local_planner_node.cpp
  src/local_planner/LocalPlanner.cpp
  src/dwa_planner/DwaPlanner.cpp
  src/dwa_planner/VelocityWindow.cpp
  src/dwa_planner/DwaPlannerRosConverter.cpp
  
)
target_link_libraries(dwa_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_planner_visualization_node src/local_planner_visualization_node.cpp
  src/local_planner_visualization/LocalPlannerVisualization.cpp
  
)
target_link_libraries(local_planner_visualization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)