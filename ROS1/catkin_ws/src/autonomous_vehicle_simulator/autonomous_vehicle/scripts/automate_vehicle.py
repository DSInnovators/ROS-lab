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

from vehicle_control import *
from road_detection import *

# Define your image topic
image_topic = "/vehicle_camera/image_raw"

normal_speed = 0.3
turn_speed = 0.15

def move_vehicle(direction):
    command_publisher = CommandPublisher()
    global key_code
    if direction == MOVE_LEFT:
        key_code = 'u'
        command_publisher.set_speed(turn_speed)
    if direction == MOVE_RIGHT:
        key_code = 'o'
        command_publisher.set_speed(turn_speed)
    if direction == TURN_LEFT:
        key_code = 'j'
        command_publisher.set_speed(turn_speed)
    if direction == TURN_RIGHT:
        key_code = 'l'
        command_publisher.set_speed(turn_speed)
    if direction == MOVE_FORWARD:
        key_code = 'i'
        command_publisher.set_speed(normal_speed)
    if direction == MOVE_STOP:
        key_code = 't'

    command_publisher.publish_command(key_code)

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect road and get the decision to move forward or turn
        road_detection = RoadDetection(image)
        decision = road_detection.process_image()

        # control vehicle
        move_vehicle(decision)
                
    except  Exception as e:        
        # print("exception: ")
        print(e)
        traceback.print_exc()


def main():
    rospy.init_node('autonomous_vehicle')

    # Set up image topic subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()