#!/usr/bin/env bash

set -x

DISTRO="$(lsb_release -sc)"
if [[ "$DISTRO" == "focal" ]]; then
    ROS_DISTRO="noetic"
elif [[ "$DISTRO" == "bionic" ]]; then
    ROS_DISTRO="melodic"
elif [[ "$DISTRO" == "xenial" ]]; then
    ROS_DISTRO="kinetic"
fi

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt install -qq ros-$ROS_DISTRO-desktop-full

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

sudo apt install python3-pip python3-yaml python3-setuptools
sudo pip3 install rosdep rosinstall rospkg catkin-pkg
sudo rosdep init
rosdep update

# AirSim ROS Wrapper dependencies

if [[ "$DISTRO" == "xenial" ]]; then
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    sudo apt-get update
fi

sudo apt-get install gcc-8 g++-8
sudo apt-get install ros-$ROS_DISTRO-mavros* ros-$ROS_DISTRO-tf2-sensor-msgs

# TODO: Remove this if-block when new 0.7.0 release of catkin_tools is available
if [[ "$DISTRO" == "focal" ]]; then
    sudo pip3 install "git+https://github.com/catkin/catkin_tools.git#egg=catkin_tools"
else
    sudo pip3 install catkin-tools
fi
