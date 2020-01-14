cmake_minimum_required(VERSION 2.8.3)
project(rpi_gpio_estop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  duckiefloat_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpi_gpio_estop
#  CATKIN_DEPENDS rospy std_msg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

