FROM ros:melodic-ros-core

MAINTAINER bensonpuppy1997

RUN apt-get update && apt-get install -y \
    build-essential \
    python-pip \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    #ros-${ROS_DISTRO}-cv-bridge \
    #ros-${ROS_DISTRO}-pcl-conversions \
    #ros-${ROS_DISTRO}-pcl-ros \
    #ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    #ros-${ROS_DISTRO}-tf-conversions \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install Adafruit-motorhat==1.4.0
    
