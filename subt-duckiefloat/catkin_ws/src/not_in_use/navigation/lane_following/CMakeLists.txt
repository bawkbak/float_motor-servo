cmake_minimum_required(VERSION 2.8.3)
project(lane_following)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  target_pose.srv
)

generate_dynamic_reconfigure_options(
    cfg/pos_PID.cfg
    cfg/ang_PID.cfg
    cfg/lookahead.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(lidar_detection src/lidar_detection.cpp)
target_link_libraries(lidar_detection ${catkin_LIBRARIES})

add_executable(rgbd_detection src/rgbd_detection.cpp)
target_link_libraries(rgbd_detection ${catkin_LIBRARIES})

add_executable(pursuit_target_node src/pursuit_target_node.cpp)
add_dependencies(pursuit_target_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(pursuit_target_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(velodyne_to_grid src/velodyne_to_grid.cpp)
add_dependencies(velodyne_to_grid ${catkin_EXPORTED_TARGETS})
target_link_libraries(velodyne_to_grid ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(wall_following_node src/wall_following_node.cpp)
add_dependencies(wall_following_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(wall_following_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(sim_ground_node src/sim_ground.cpp)
add_dependencies(sim_ground_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(sim_ground_node ${catkin_LIBRARIES})
