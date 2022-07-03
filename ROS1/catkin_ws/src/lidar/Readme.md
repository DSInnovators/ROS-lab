## Depenencies
```sh
# For patchwork
$ sudo apt-get install ros-noetic-pcl-conversions
$ sudo apt-get install ros-noetic-pcl-ros
$ sudo apt-get install ros-noetic-jsk-recognition
$ sudo apt-get install ros-noetic-jsk-common-msgs
$ sudo apt-get install ros-noetic-jsk-rviz-plugins

```

## Running Scripts
```sh
# Running Velodyne Launch Files
$ roslaunch lidar_tutorial velodyne_lidar.launch

# Running Z indexed segmentation
$ rosrun patchwork z_indexed_segmentation 

# Running PCL Segmentation
$ rosrun patchwork lidar_pcl_segmentation

```