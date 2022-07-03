## Running scripts
```sh

# Python
$ roslaunch gazebo_vehicle_contorl gazebo_vehicle_control.launch
$ python3 object_avoiding.py
$ python3 object_detecting.py

# CPP
$ roslaunch gazebo_vehicle_contorl gazebo_vehicle_control.launch
$ rosrun object_avoidance object_avoidance
```





## Install Dependencies
```sh
$ pip3 install torchvision
$ pip3 install imutils


```

## Check OpenCV Version
```sh
  std::cout<< cv::getBuildInformation().c_str()<<"\n";
```

# References
1. https://learnopencv.com/object-detection-using-yolov5-and-opencv-dnn-in-c-and-python/
2. https://medium.com/mlearning-ai/detecting-objects-with-yolov5-opencv-python-and-c-c7cf13d1483c
