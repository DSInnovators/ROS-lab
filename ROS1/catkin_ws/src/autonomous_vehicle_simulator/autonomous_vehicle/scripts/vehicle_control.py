
import rospy
from geometry_msgs.msg import Twist

MOVE_LEFT       = "move_left"
MOVE_RIGHT      = "move_right"
MOVE_FORWARD    = "move_forward"
MOVE_STOP       = "stop"
TURN_LEFT       = "turn_left"
TURN_RIGHT      = "turn_right"

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
        self.speed = rospy.get_param("~speed", 0.3)
        self.turn = rospy.get_param("~turn", 0.1)
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

        # debug print
        print(twist)

        # Publish.
        self.publisher.publish(twist)


    def set_speed(self, speed):
        # print(key)
        self.speed = speed


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)