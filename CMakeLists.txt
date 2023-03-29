cmake_minimum_required(VERSION 3.0.0)
project(merge_points VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3  -pthread -fopenmp")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}" ${CMAKE_MODULE_PATH})
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_BUILD_TYPE Release)
find_package(PCL 1.8 REQUIRED)

# ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag             # use rosbag::Bag
  sensor_msgs
  message_filters
  geometry_msgs
  nav_msgs
  visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(merge_points_lib SHARED 
                  tf_subscriber.hpp 
                  cloud_subscriber.hpp)

add_executable(${PROJECT_NAME} ros_node_test.cpp) 
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES} merge_points_lib)