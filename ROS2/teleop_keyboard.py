# Ref: https://github.com/ros2/teleop_twist_keyboard
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)

import sys

import geometry_msgs.msg
import rclpy

import click


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
           UP   
   LEFT  SPACE  RIGHT
          DOWN  

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

# \x1b[A UP arrow
# \x1b[B DOWN arrow
# \x1b[C RIGHT arrow
# \x1b[D LEFT arrow

up = '\x1b[A'
down = '\x1b[B'
right = '\x1b[C'
left = '\x1b[D'
space = " "

moveBindings = {
    up: (1, 0, 0, 0),
    right: (1, 0, 0, -1),
    left: (1, 0, 0, 1),
    down: (-1, 0, 0, 0),
    space: (0, 0, 0, 0)
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)

        if(key == '\033'):
            key += sys.stdin.read(2)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
