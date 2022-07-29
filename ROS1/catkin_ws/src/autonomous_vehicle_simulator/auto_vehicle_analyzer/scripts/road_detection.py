#!/usr/bin/env python3
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
import numpy as np
import traceback

from vehicle_control import *
from auto_vehicle_msgs.msg import Status
import sys, select, termios, tty

SLOPE_1 = 1.0
SLOPE_2 = 1.2
SLOPE_3 = 1.5
SLOPE_4 = 1.8


command_vehicle = 'vehicle_data'

class RoadDetection:
    def __init__(self):
        self.image = None
        self.detection_model = None
        self.slope_file = open("slope.txt", 'w')
        self.publisher = rospy.Publisher(command_vehicle, Status, queue_size=1000)
    #
    # def grey(self, image):
    #   #convert to grayscale
    #     image = np.asarray(image)
    #     return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #
    #   #Apply Gaussian Blur --> Reduce noise and smoothen image
    # def gauss(self, image):
    #     return cv2.GaussianBlur(image, (5, 5), 0)
    #
    #   #outline the strongest gradients in the image --> this is where lines in the image are
    # def canny(self, image):
    #     edges = cv2.Canny(image,50,150)
    #     return edges

    def set_image(self, road_image):
        self.image = road_image

      def get_isolated_region(self, image):
        # global width
        # global height
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

    def get_isolated_region_for_lane(self, image):
        # global width
        # global height
        height, width = image.shape
        # isolate the gradients that correspond to the lane lines
        # specify coordinates of the polygon
        polygon = np.array([[0, height], [220, 460], [660, 400], [width, height]])

        # # fill polygon with ones
        # cv2.fillConvexPoly(stencil, polygon, 1)
        # create a black image with the same dimensions as original image
        mask = np.zeros_like(image)
        # create a mask (triangle that isolates the region of interest in our image)
        mask = cv2.fillConvexPoly(mask, polygon, 255)
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
        # isolated_region = self.get_isolated_region(edges)
        isolated_region = self.get_isolated_region_for_lane(edges)

        # Hough Transform, to find those white pixels from isolated region into actual lines
        # DRAWING LINES: (order of params) --> region of interest, bin size (P, theta), min intersections needed, placeholder array,
        lines = cv2.HoughLinesP(isolated_region, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)

        decision = self.get_decision_from_average(copy, lines)
        # decision = self.get_decision_after_obstacle_found(self.image)
        return decision

    def display_lines(self, image, lines):
        lines_image = np.zeros_like(image)
        #make sure array isn't empty
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line
                #draw lines on a black image
                cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
                # print("x1 {} x2 {}".format(x1,x2))
        return lines_image

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

    def get_decision_from_average(self, image, lines):
        left = []
        right = []

        vehicle_msg = Status()

        speed = SPEED_1
        turn = TURN_1
        slope_abs = 0.0



        # image_for_obstacle = np.copy(image)
        # obs_avoidance = ObstacleAvoidance()
        # decision = obs_avoidance.find_obstacle(image_for_obstacle)
        # if decision == TURN_LEFT or decision == TURN_RIGHT:
        #     print("-------- turn for obstacle ------ {}".format(decision))
        # # decision = self.get_decision_after_obstacle_found(image_for_obstacle)
        #     return decision
        # if decision == MOVE_LEFT or decision == MOVE_RIGHT:
        #     print("-------- turn for obstacle ------")
        #     # print(decision)
        #     #     # todo temporary disable
        #     return decision
        # decision = MOVE_STOP
        if lines is not None:
            for line in lines:
                # print(line)
                x1, y1, x2, y2 = line.reshape(4)
                # fit line to points, return slope and y-int
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                # print("parameters {}".format(parameters))
                # print("road x1 {} x2 {}".format(x1,x2))
                slope = parameters[0]
                y_int = parameters[1]
                print("road x1 {} x2 {} slope {:.3f} y_int {}".format(x1, x2, slope, y_int))
                # lines on the right have positive slope, and lines on the left have neg slope
                # test code
                slope_abs = abs(slope)
                # if slope_abs < SLOPE_1:
                #     CommandPublisher.set_speed(SPEED_1)
                #     CommandPublisher.set_turn(TURN_1)
                #     print("------ speed1 {} turn1 {} ------".format(SPEED_1, TURN_1))
                # elif abs(slope) < SLOPE_2:
                #     # CommandPublisher.set_speed(SPEED_2)
                #     # CommandPublisher.set_turn(TURN_2)
                #     print("------ speed1 {} turn1 {} ------".format(SPEED_2, TURN_2))
                # elif 1.0 < abs(slope) < SLOPE_3:
                #     # CommandPublisher.set_speed(SPEED_3)
                #     # CommandPublisher.set_turn(TURN_3)
                #     print("------ speed2 {} turn2 {} ------".format(SPEED_3, TURN_3))
                if SLOPE_1 < slope_abs < SLOPE_4:
                    speed = float("{:.3f}".format(SPEED_1/ slope_abs)) #SPEED_1/ abs(slope)
                    # turn =  TURN_3 #float("{:.2f}".format(TURN_2 * slope_abs)) #TURN_2 * abs(slope)
                    turn = float("{:.3f}".format(slope_abs/10));
                    # CommandPublisher.set_speed(SPEED_1/ abs(slope))
                    # CommandPublisher.set_turn(TURN_2 * abs(slope))
                    print("------ speed2 {} turn2 {} ------".format(SPEED_2/ abs(slope), TURN_2 * abs(slope)))
                    # pass
                elif slope_abs > SLOPE_4:
                    speed = SPEED_4
                    turn = TURN_4 # float("{:.2f}".format(slope_abs/10))
                    print("------ slope_abs -----------------############ {:.3f} ".format(slope_abs))
                    # slope_abs = SLOPE_4
                    # CommandPublisher.set_speed(SPEED_4)
                    # CommandPublisher.set_turn(TURN_4)

                print(" in loop------ slope {:.3f} speed {:.3f} turn {:3f} ------".format(slope_abs, speed, turn))
                # vehicle_msg.curtime = round(time.time() * 1000)
                # vehicle_msg.slope = slope_abs
                # vehicle_msg.turn = turn
                # vehicle_msg.speed = speed
                #
                # self.publisher.publish(vehicle_msg)
                # rospy.loginfo(vehicle_msg)
                #
                # CommandPublisher.set_speed(speed)
                # CommandPublisher.set_turn(turn)
                output_file2 = self.slope_file
                output_file2.write("{:.2f},{:.2f},{:.2f}\n".format(slope_abs, speed * 10, turn))
                # now = round(time.time() * 1000)
                CommandPublisher.set_speed(speed)
                CommandPublisher.set_turn(turn)
                # if slope < -1.0:
                #     print("------ move right slope ------")
                #     return MOVE_RIGHT
                # elif slope > 1.0:
                #     print("------ move left slope ------")
                #     return MOVE_LEFT
                # else :
                #     print("====== move forward slope ======")
                #     # return MOVE_FORWARD
                # test code end

                if slope < 0:
                    left.append((slope, y_int))
                else:
                    right.append((slope, y_int))

        # print("left {} right {}".format(left, right))
        # takes average among all the columns (column0: slope, column1: y_int)
        right_avg = np.average(right, axis=0)
        left_avg = np.average(left, axis=0)
        # print(type(left_avg))
        # print(type(right_avg))
        # print("x1 {} x2 {}".format(x1, x2))
        print("left_avg {} right_avg {}".format(left_avg, right_avg))
        # print("-------- search for obstacle ------")
        # Make a copy of resized image

        if isinstance(left_avg, np.ndarray) and np.count_nonzero(right_avg > 0):
            if isinstance(right_avg, np.ndarray) and np.count_nonzero(right_avg > 0):
                pass
        # speed = SPEED_1
        # turn = TURN_1
        # speed = SPEED_2
        # turn = TURN_2
        elif slope_abs == 0.0:
            print("------ slope 0 -----------------############ ")
            speed = SPEED_3
            turn = TURN_3

        CommandPublisher.set_speed(speed)
        CommandPublisher.set_turn(turn)
        print("------ slope {:.3f} speed {:.3f} turn {:3f} ------".format(slope_abs, speed, turn))

        # vehicle_msg.curtime = round(time.time() * 1000)
        # vehicle_msg.slope = slope_abs
        # vehicle_msg.turn = turn
        # vehicle_msg.speed = speed
        # self.publisher.publish(vehicle_msg)



        if isinstance(left_avg, np.ndarray) and np.count_nonzero(left_avg == 0):  # or math.isnan(left_avg):
            print("------ move left for ndarray edge ------")
            # move_vehicle(TURN_LEFT)
            # decision = decision | MOVE_LEFT
            # CommandPublisher.set_speed(SPEED_3)
            # CommandPublisher.set_turn(TURN_3)

            vehicle_msg.curtime = round(time.time() * 1000)
            vehicle_msg.slope = slope_abs
            vehicle_msg.turn = turn
            vehicle_msg.speed = speed
            self.publisher.publish(vehicle_msg)
            rospy.loginfo(vehicle_msg)
            return  MOVE_LEFT
        elif isinstance(left_avg, np.float64) and math.isnan(left_avg):
            print(" ------ move left for nan edge ---- ")
            # move_vehicle(TURN_LEFT)
            # decision = decision | MOVE_LEFT
            # CommandPublisher.set_speed(SPEED_3)
            # CommandPublisher.set_turn(TURN_3)

            vehicle_msg.curtime = round(time.time() * 1000)
            vehicle_msg.slope = slope_abs
            vehicle_msg.turn = turn
            vehicle_msg.speed = speed
            self.publisher.publish(vehicle_msg)
            rospy.loginfo(vehicle_msg)

            return MOVE_LEFT
        elif isinstance(right_avg, np.ndarray) and np.count_nonzero(right_avg == 0):  # or math.isnan(right_avg):
            print("----- move right for ndarray edge -----")
            # move_vehicle(TURN_RIGHT)
            # decision = decision | MOVE_RIGHT
            # CommandPublisher.set_speed(SPEED_3)
            # CommandPublisher.set_turn(TURN_3)

            vehicle_msg.curtime = round(time.time() * 1000)
            vehicle_msg.slope = slope_abs
            vehicle_msg.turn = turn
            vehicle_msg.speed = speed
            self.publisher.publish(vehicle_msg)
            rospy.loginfo(vehicle_msg)
            return MOVE_RIGHT
        elif isinstance(right_avg, np.float64) and math.isnan(right_avg):
            print("-------- move right for nan edge ------")
            # move_vehicle(TURN_RIGHT)
            # decision = decision | MOVE_RIGHT
            # CommandPublisher.set_speed(SPEED_3)
            # CommandPublisher.set_turn(TURN_3)

            vehicle_msg.curtime = round(time.time() * 1000)
            vehicle_msg.slope = slope_abs
            vehicle_msg.turn = turn
            vehicle_msg.speed = speed
            self.publisher.publish(vehicle_msg)
            rospy.loginfo(vehicle_msg)
            return MOVE_RIGHT
        else:
            speed = SPEED_1
            turn = TURN_1
            CommandPublisher.set_speed(speed)
            CommandPublisher.set_turn(turn)
            print("----- move forward slope {:.3f} speed {:.3f} turn {:3f} ------".format(slope_abs, speed, turn))
            vehicle_msg.curtime = round(time.time() * 1000)
            vehicle_msg.slope = slope_abs
            vehicle_msg.turn = turn
            vehicle_msg.speed = speed
            self.publisher.publish(vehicle_msg)
            rospy.loginfo(vehicle_msg)
            # create lines based on averages calculates
            left_line = self.make_points(image, left_avg)
            right_line = self.make_points(image, right_avg)
            # print("x1 {} right x2 {}".format(left_line[0], right_line[0]))
            # if left_line[0] < 50:
            #     print("------------ move to right -----------------")
            #     # move_vehicle(TURN_RIGHT)
            #     return TURN_RIGHT
            # elif right_line[0] > 600:
            #     print("---------- move to left ------------------")
            #     # move_vehicle(TURN_LEFT)
            #     return TURN_LEFT
            # else:
            # print("left_line {} right_line {}".format(left_line, right_line))
            print("-------- move forward --------- ")
            # move_vehicle(MOVE_FORWARD)
            # decision = decision | MOVE_FORWARD
            return MOVE_FORWARD
            # return None  # np.array([left_line, right_line])

            # return self.get_decision_after_obstacle_found(image)

        return decision

    def get_decision_after_obstacle_found(self, image):
        thres = 0.45 # Threshold to detect object
        classNames = []
        classFile = 'coco.names'
        with open(classFile,'rt') as f:
            classNames = f.read().rstrip('\n').split('\n')

        # configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        # weightsPath = 'frozen_inference_graph.pb'

        # detection_model = cv2.dnn_DetectionModel(weightsPath, configPath)
        # detection_model.setInputSize(320, 320)
        # detection_model.setInputScale(1.0/ 127.5)
        # detection_model.setInputMean((127.5, 127.5, 127.5))
        # detection_model.setInputSwapRB(True)
        # #
        class_ids, confs, bbox = RoadDetection.detection_model.detect(image, confThreshold = thres)
        # return None
        if len(class_ids) != 0:
            for class_id, confidence, box in zip(class_ids.flatten(), confs.flatten(), bbox):
                # cv2.rectangle(image, box, color=(0, 255, 0), thickness=2)
                (startX, startY, endX, endY) = box.astype("int")
                # print(startX, endX)
                str_confidence = str(round(confidence * 100, 2))

                if confidence > 0.50:
                    # cv2.putText(image, classNames[class_id - 1].upper(), (box[0] + 10, box[1] + 30),
                    #             cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    # cv2.putText(image, str_confidence, (box[0] + 200, box[1] + 30),
                    #             cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    # # print(box)
                    # print(classNames[class_id - 1])
                    print("------- obstacle prediction {}, confidence {}  -----".format(
                         classNames[class_id - 1], str(round(confidence * 100, 2)), ))
                if confidence > 0.70:
                    print("============= obstacle found {}, confidence {}  x1 {} x2 {} ===============".format(
                        classNames[class_id - 1], str_confidence, startX, endX))
                        # cv2.imshow("Obstacle", image)
                        # cv2.waitKey(2)
                    x1 = startX #min(startX, endX)
                    if x1 >= 150 :  # go left
                        print("left turn")
                        return MOVE_LEFT
                    elif 1 <= x1 < 150:
                        print("right turn")
                        return  MOVE_RIGHT
                    else:
                        return MOVE_FORWARD
                    # return  MOVE_STOP
        else:
            return MOVE_FORWARD
