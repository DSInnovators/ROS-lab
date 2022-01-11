# ROS2 and Gazebo

### Environment setup for Ubuntu 20.04

0. Download *install_ros2.sh* in your computer.
1. Open a terminal and run <tt>sudo chmod +x install_ros2.sh</tt>.
2. Execute the bash file by running <tt>sudo ./install_ros2.sh</tt>.
3. Source the bashrc file <tt>source ~/.bashrc</tt>.

### Check Environment Variables

```
printenv | grep -i ROS
```

The response should be -

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=foxy
```

## Keyboard Controlled Differential Drive Robot with ROS2 and Gazebo

### Install gazebo_ros_pkgs
Gazebo is installed alongside ROS2. In order to install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros), run the following command: 
```
sudo apt install ros-foxy-gazebo-ros-pkgs
```
Make sure you have some core tools installed: 
```
sudo apt install ros-foxy-ros-core ros-foxy-geometry2
```
To test if the **gazebo_ros_pkgs** was successfully installed, open a new terminal, source ROS2, and simulate a world file using the following command:
```
gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```

### Build custom plugin for differential drive robot

We will build a custom plugin and refer to that **.so** object in our world file. The world file is located in this repo under [*ROS2/world/gazebo_ros_diff_drive_demo.world*](https://github.com/DSInnovators/ROS-lab/blob/main/ROS2/world/gazebo_ros_diff_drive_demo.world). 

1. Clone the repo `git clone git@github.com:DSInnovators/ROS-lab.git`
2. Navigate to `ROS2/gazebo-ros-plugins/`
3. Run the following commands:
```
mkdir build # create a build folder
cd build # go to build folder
cmake ../ # compile
make
```
4. A **.so** file will be created inside the *build* folder, namely `libgazebo_ros_diff_drive.so`.
5. Copy and paste the folder to a location where the **.world** file will be able to refer it from
```
rm -rf /opt/ros/foxy/lib/libgazebo_ros_diff_drive.so # remove the previously fetched (with ros-gazebo-ros-pkgs) diff drive .so file 
cp ~/PATH_TO_REPO/ROS2/gazebo-ros-plugins/build/libgazebo_ros_diff_drive.so /opt/ros/foxy/lib/libgazebo_ros_diff_drive.so 
```
Use `sudo` command if necessary.
6. Open a terminal, run the world file:
```
cd ~/PATH_TO_REPO/ROS2/world
gazebo --verbose gazebo_ros_diff_drive_demo.world
```
A gazebo window will load with a differential drive robot.
7. Open another terminal and run the publisher:
```
cd ~/PATH_TO_REPO/ROS2/
python3 teleop_keyboard.py
```
8. Run the following command to check if the subscriber and publisher are running smoothly:
```
ros2 topic in --verbose /cmd_vel # cmd_vel is the name of the topic the publisher is publishing to and the subscriber is subscribing to
```
You should see a response like this:
```
Type: geometry_msgs/msg/Twist

Publisher count: 1

Node name: teleop_twist_keyboard
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: PUBLISHER
GID: 01.0f.a2.8b.b8.cd.c9.d9.01.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 1

Node name: diff_drive
Node namespace: /
Topic type: geometry_msgs/msg/Twist
Endpoint type: SUBSCRIPTION
GID: 01.0f.a2.8b.11.ce.02.62.01.00.00.00.00.00.13.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds
```





