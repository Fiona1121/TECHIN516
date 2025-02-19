#!/bin/bash

# update before installing anything
sudo apt update

source /opt/ros/humble/setup.bash
sudo rosdep init
rosdep update

# install python packages
sudo apt install -y python3-pip
pip install matplotlib
pip install pandas
pip install numpy
pip install pyquaternion

# install helpful tools
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt-* \
    net-tools

# install gazebo
sudo apt install -y \
    ros-humble-gazebo-*
echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc

# install kinova dependencies
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-colcon-clean \
    python3-vcstool \
    ros-humble-moveit \
    ros-humble-realtime-tools \ 
pip install moveit_config_utils

# create src directory
mkdir src

# # ros2 control
sudo apt-get install -y wget
vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos src
rosdep update --rosdistro=$ROS_DISTRO

# # install kinova packages
git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex
cd src/ros2_kortex
git checkout 97a0e7c
cd ../..
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos
cd src
rm -rf control_msgs gz_ros2_control ros2_controllers gazebo_ros2_control ros2_control 
cd ..
sudo apt-get update
rosdep install --ignore-src --from-paths src -y -r

# install pymoveit2
cd src
git clone https://github.com/AndrejOrsula/pymoveit2.git
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths ./pymoveit2
cd ..

# build
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
source install/setup.bash
