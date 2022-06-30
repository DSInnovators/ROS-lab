#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from datetime import datetime
import time

from matplotlib import pyplot as plt


MOVE_LEFT  = 0x01 #"move_left"
MOVE_RIGHT = 0x02 #"move_right"
MOVE_FORWARD = 0x03 #"move_forward"
MOVE_STOP = 0x00 #"stop"
TURN_LEFT = 0x004#"turn_left"
TURN_RIGHT = 0x008#"turn_right"

SPEED_1 = 0.4
SPEED_2 = 0.3
SPEED_3 = 0.2
SPEED_4 = 0.15
SPEED_5 = 0.1

TURN_1 = 0.1
TURN_2 = 0.1
TURN_3 = 0.12
TURN_4 = 0.15
TURN_5 = 0.18

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

command = 'vehicle/cmd_vel'

class CommandPublisher:

    def __init__(self):
        # self.speed = rospy.get_param("~speed", 0.3)
        # self.turn = rospy.get_param("~turn", 0.15)
        self.repeat = rospy.get_param("~repeat_rate", 1.0)
        key_timeout = rospy.get_param("~key_timeout", 0.0)
        if key_timeout == 0.0:
            key_timeout = None
        self.publisher = rospy.Publisher(command, Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.done = False
        self.time_speed_file = open("time_speed.txt", 'w')
        # self.speed_file = open("speed.txt", 'w')

    def publish_command(self, key):
        # print(key)
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
        elif key in speedBindings.keys():
            self.speed = speed * speedBindings[key][0]
            self.turn = turn * speedBindings[key][1]
   
        twist = Twist()
        # Copy state into twist message.
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th * self.turn
        # print(twist)
        # Publish.

        now = round(time.time() * 1000) #datetime.now().time()  # time object
        output_file = self.time_speed_file
        output_file.write("{},{}\n".format(now, self.speed*10))
        # output_file2 = self.speed_file
        # output_file2.write("{}\n".format(self.speed*10))
        # print(" ==== time {} speed {} ========== ".format(now, self.speed))

        self.publisher.publish(twist)

        # print("=== after publish ===")
    @staticmethod
    def set_speed(speed):
        # print(key)
        CommandPublisher.speed = speed

    @staticmethod
    def set_turn(turn):
        CommandPublisher.turn = turn

    # def update(self, x, y, z, th, speed, turn):
    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.th = th
    #     self.speed = speed
    #     self.turn = turn
    #     # Notify publish thread that we have a new message.
    #
    # def stop(self):
    #     self.done = True
    #     self.update(0, 0, 0, 0, 0, 0)
    #     self.join()
    #
    # def run(self):
    #     twist = Twist()
    #         # Copy state into twist message.
    #     twist.linear.x = self.x * self.speed
    #     twist.linear.y = self.y * self.speed
    #     twist.linear.z = self.z * self.speed
    #     twist.angular.x = 0
    #     twist.angular.y = 0
    #     twist.angular.z = self.th * self.turn
    #
    #     # Publish.
    #     self.publisher.publish(twist)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)