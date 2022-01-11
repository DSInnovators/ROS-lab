# Successfully tested on Ubuntu 20.04

## Enviroment Configuration
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init && rosdep update
```

## Check if perfectly configured
```sh printenv | grep ROS```

#### It should print like this:

```sh
ROS_VERSION=1
ROS_PYTHON_VERSION=3
ROS_PACKAGE_PATH=/opt/ros/noetic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_MASTER_URI=http://localhost:11311
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_DISTRO=noetic
```

## Create Workspace 
#### (if new package needed to be created, not required for this one)
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Navigating the ROS Filesystem
 
```sudo apt-get install ros-noetic-ros-tutorials```

## Run Project
1. Open first terminal and paste command `roscore` and run
2. Open second terminal for publisher and paste command `source ./devel/setup.bash && rosrun beginner_tutorials talker.py`
2. Open third terminal for subscriber and paste command `source ./devel/setup.bash && rosrun beginner_tutorials listener.py`

