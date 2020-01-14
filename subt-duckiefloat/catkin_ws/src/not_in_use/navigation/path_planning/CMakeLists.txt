cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(path_planning src/path_planning.cpp)
target_link_libraries(path_planning ${catkin_LIBRARIES})

add_executable(custom_a_star src/custom_a_star.cpp)
target_link_libraries(custom_a_star ${catkin_LIBRARIES})