#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html


from __future__ import print_function
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import traceback

import sys
import numpy as np
import math
import random

import gym
import gym_game

from q_learning import *
from vehicle_control import *


# Define your image topic
image_topic = "/vehicle_camera/image_raw"


def main():
    rospy.init_node('autonomous_vehicle')
    
    init_rl()
    simulate()

    # Set up image topic subscriber and define its callback
    # rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()