#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
from __future__ import print_function
import math

import rospy
# ROS Image message
# from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
# from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from matplotlib import pyplot as plt
import numpy as np
import traceback

from vehicle_control import *

import sys, select, termios, tty

import sys
import numpy as np
import math
import random

import gym


from os.path import exists


class RoadDetection:
    def __init__(self, road_image):
        self.image = road_image

    def get_isolated_region(self, image):
        height, width = image.shape
        #isolate the gradients that correspond to the lane lines
        triangle = np.array([
                           [(0, height), (int(width/2), int(height/2)), (width, height)]
                           ])
        #create a black image with the same dimensions as original image
        mask = np.zeros_like(image)
        #create a mask (triangle that isolates the region of interest in our image)
        mask = cv2.fillPoly(mask, triangle, 255)
        mask = cv2.bitwise_and(image, mask)
        return mask

    def process_image(self):
        # percent by which the image is resized
        scale_percent = 100

        # calculate the 50 percent of original dimensions
        width = int(self.image.shape[1] * scale_percent / 100)
        height = int(self.image.shape[0] * scale_percent / 100)

        # dsize
        dsize = (width, height)

        # resize image
        resized_image = cv2.resize(self.image, dsize)
        cv2.imwrite("canny_edges_resize.png", resized_image)

        # Make a copy of resized image
        copy = np.copy(resized_image)

        # find the edges
        edges = cv2.Canny(copy, 50, 150)

        # isolate a certain region in the image where the lane lines are
        isolated_region = self.get_isolated_region(edges)

        # Hough Transform, to find those white pixels from isolated region into actual lines
        # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
        lines = cv2.HoughLinesP(isolated_region, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)

        # get the road state
        road_state = self.get_road_state(copy, lines)
        
        return road_state
    
    def get_road_state(self, image, lines):
        left = []
        right = []
        slope = 0
        if lines is None:
            # print("===== line is None ===== ")
            # return 0 #[0, 0]
            pass

        if lines is not None:
            if np.array_equal(lines, []) == True:            
                print("===== line is empty ===== ")
                return 0 # [0, 0]
            
            for line in lines:
                # print(line)
                x1, y1, x2, y2 = line.reshape(4)
                if x1 == x2:
                    print("===== slope infinite ===== ")
                    return 0 # [0,0]
                if y1 == y2:
                    print("===== slope 0 ===== ")
                    return 0  # [0,0]
                # fit line to points, return slope and y-int
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                # print(parameters)
                # print("x1 {} x2 {}".format(x1,x2))
                slope = parameters[0]
                y_int = parameters[1]
                slope_abs = abs(slope)
                if slope_abs > 7: #2.5:
                    print("-------higher slope-------------", slope_abs)
                    return 0               
                # # lines on the right have positive slope, and lines on the left have neg slope
                if slope < 0:
                    left.append((slope, y_int))
                else:
                    right.append((slope, y_int))
            

        # takes average among all the columns (column0: slope, column1: y_int)
        right_avg = np.average(right, axis=0)
        left_avg = np.average(left, axis=0)

        if isinstance(left_avg, np.ndarray) and np.count_nonzero(left_avg == 0):
            # print("------ move left ndarray------", slope)
            return 1#[0, 1] # return MOVE_LEFT
        elif isinstance(left_avg, np.float64) and math.isnan(left_avg):
            # print(" ------ move left for nan ---- ", slope)
            return 1 #[0, 1] # return MOVE_LEFT
        elif isinstance(right_avg, np.ndarray) and np.count_nonzero(right_avg == 0):
            # print("----- move right ndarray -----", slope)
            return 2 # [1, 0] # return MOVE_RIGHT
        elif isinstance(right_avg, np.float64) and math.isnan(right_avg):
            # print("-------- move right nan------", slope)
            return 2 #[1, 0] # return MOVE_RIGHT
        else:
            # create lines based on averages calculates
            left_line = self.make_points(image, left_avg)
            right_line = self.make_points(image, right_avg)
            
            return 3 #[1, 1] # return MOVE_FORWARD

        return None
    

    def make_points(self, image, average):
        # print(average)
        slope, y_int = average
        y1 = image.shape[0]
        # how long we want our lines to be --> 3/5 the size of the image
        y2 = int(y1 * (3 / 5))
        # determine algebraically
        x1 = int((y1 - y_int) // slope)
        x2 = int((y2 - y_int) // slope)
        # print("x1 {} y1 {}".format(x1,x2))
        return np.array([x1, y1, x2, y2])
    
  
    
 