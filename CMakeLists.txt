cmake_minimum_required(VERSION 3.0.2)
project(map_wise)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()




###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_wise
  CATKIN_DEPENDS geometry_msgs livox_ros_driver2 nav_msgs pcl_conversions pcl_ros roscpp roslib sensor_msgs std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########
link_directories(/usr/local/lib)

include_directories(
  src/nano_flann
  src/utility
  src/cloud_process
  src/p2pl_icp
  src/eskf
  src/gps
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  /usr/local/include/yaml-cpp
)

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_flann/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

add_executable(livox_process_node 
src/cloud_process/livox_process_node.cpp
src/cloud_process/livox_process.cpp
src/utility/utility.cpp
)
add_dependencies(livox_process_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_process_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)


add_executable(location_node 
src/location.cpp
src/p2pl_icp/p2pl_icp.cpp
src/utility/utility.cpp
src/eskf/eskf.cpp
)
add_dependencies(location_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(location_node ${catkin_LIBRARIES} ${OpenMP_LIBS} ${PCL_LIBRARIES} yaml-cpp nanoflann)

add_executable(location_with_gps_node 
src/location_with_rtk.cpp
src/p2pl_icp/p2pl_icp.cpp
src/utility/utility.cpp
src/eskf/eskf.cpp
src/gps/gps.cpp
)
add_dependencies(location_with_gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(location_with_gps_node ${catkin_LIBRARIES} ${OpenMP_LIBS} ${PCL_LIBRARIES} yaml-cpp nanoflann)


add_executable(lidar_calib_node 
src/lidar_calib.cpp
src/p2pl_icp/p2pl_icp.cpp
src/utility/utility.cpp
)
add_dependencies(lidar_calib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_calib_node ${catkin_LIBRARIES} ${OpenMP_LIBS} ${PCL_LIBRARIES} yaml-cpp nanoflann)