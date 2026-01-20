cmake_minimum_required(VERSION 3.0.2)
project(blind_spot_obstacle_layer)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             visualization_msgs
             message_filters
             laser_geometry
             sensor_msgs
             costmap_2d
             pluginlib
             std_srvs
             tf2_ros
             roscpp)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/BlindSpotObstacleLayer.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
  dynamic_reconfigure
  visualization_msgs
  message_filters
  laser_geometry
  sensor_msgs
  costmap_2d
  pluginlib
  std_srvs
  tf2_ros
  roscpp
  DEPENDS
  EIGEN3
  Boost)

add_library(blind_spot_obstacle_layer src/blind_spot_obstacle_layer.cpp)
add_dependencies(blind_spot_obstacle_layer ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(blind_spot_obstacle_layer
  ${catkin_LIBRARIES}
)
