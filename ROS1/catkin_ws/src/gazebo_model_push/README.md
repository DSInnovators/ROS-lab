## Simple Gazebo Model Plugin 

#### Introduction

This is a Model type plugin which is attached to a model in Gazebo.
Plugins allow complete access to the physical properties of models and their underlying elements (links, joints, collision objects). The following plugin will apply a linear velocity to its parent model.


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
To install test-automation, git clone the repo

    $ git clone https://github.com/DSInnovators/ROS-lab.git
    $ cd ROS-lab/ROS1/catkin_ws
    $ catkin_make
    $ source devel/setup.sh 
    $ gzserver -u src/gazebo_model_push/worlds/model_push.world --verbose       

In separate terminal, start the gui

    $ gzclient
   

#### Output
Click on the play button in the gui to unpause the simulation, and you should see the box move.
    

   


