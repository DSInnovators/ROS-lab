#!/bin/sh

#set locale that supports UTF-8
locale
apt update
apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#verify
locale

#setup sources
apt update
apt install curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

#install ros2 packages
apt update
apt install ros-foxy-desktop

#add sourcing to shell startup script
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc