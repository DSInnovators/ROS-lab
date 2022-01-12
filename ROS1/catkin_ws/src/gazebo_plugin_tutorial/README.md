## Simple Hello World Gazebo Plugin 

#### Introduction

This is a World type plugin which is attached to a world.
Plugins are designed to be simple. A bare bones world plugin contains a class with a few member functions.

#### Environment Setup
Installing **ROS Noetic** at *Ubuntu 20.04.3 LTS*
    
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    sudo apt install curl # if you haven't already installed curl
     
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
     
    sudo apt update
    sudo apt install ros-noetic-desktop-full
     
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
     
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
     
    sudo rosdep init
    rosdep update
 
Now check

    $ printenv | grep ROS


#### Dependencies
First, if you installed Gazebo from debians, make sure you've installed the Gazebo development files. If you installed Gazebo from source, you can ignore this step. If you have a release other than gazebo11, replace 11 with whatever version number you have.

    $ sudo apt-get install libgazebo11-dev
    


#### Installing and running the project
To install plugin project, git clone the repo

    $ git clone https://github.com/DSInnovators/ROS-lab.git
    $ cd ROS-lab/ROS1/catkin_ws
    $ catkin_make
    $ source devel/setup.sh 
    $ gzserver src/gazebo_plugin_tutorial/worlds/hello.world --verbose       
   

#### Output
You should see output similar to:

    Gazebo multi-robot simulator, version 11.9.1
    Copyright (C) 2012 Open Source Robotics Foundation.
    Released under the Apache 2 License.
    http://gazebosim.org
    
    [Msg] Waiting for master.
    [Msg] Connected to gazebo master @ http://127.0.0.1:11345
    [Msg] Publicized address: 192.168.1.30
    [Msg] Loading world file [/home/mazharul/code/ros_code/ROS-lab/ROS1/catkin_ws/src/gazebo_plugin_tutorial/worlds/hello.world]
    ====== Hello World from Plugin! ======
    

   


