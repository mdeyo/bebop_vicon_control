#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Empty, String, Float64, Bool
from geometry_msgs.msg import TransformStamped

import sys
import select
import termios
import tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


goal_location = None


def publishGoals(goal_location):
    print('Set goal to: ' + str(goal_location))
    pub_x_goal.publish(Float64(goal_location[0]))
    pub_y_goal.publish(Float64(goal_location[1]))
    pub_z_goal.publish(Float64(goal_location[2]))
    pub_yaw_goal.publish(Float64(goal_location[3]))
    pub_enable_goal.publish(Bool(True))


turtle_pad_x = 0
turtle_pad_y = 0


def turtle_pad_callback(data):
    global turtle_pad_x, turtle_pad_y
    turtle_pad_x = data.transform.translation.x
    turtle_pad_y = data.transform.translation.y


def waitForKeyPress():

    global goal_location, turtle_pad_x, turtle_pad_y

    empty = Empty()
    off = Bool(False)

    # default enable goal False
    pub_enable_goal.publish(off)

    while True:
        key = getKey()
        pub_enable_auto_land.publish(Bool(False))

        if key == '\x03':  # control-c!
            print('### Reset ###')
            goal_location = None
            pub_emergency.publish(empty)
            pub_enable_goal.publish(off)
            break

        elif key == '0':
            print('### Reset ###')
            goal_location = None
            pub_emergency.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '1':
            print('**Takeoff**')
            goal_location = None
            pub_takeoff.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '2':
            print('**Landing**')
            goal_location = None
            pub_land.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '3':
            goal_location = [-1.0, 1.0, 0.75, 1]
            publishGoals(goal_location)

        elif key == '4':
            goal_location = [-1.0, 1.0, 1.3, 1]
            publishGoals(goal_location)

        elif key == '5':
            goal_location = [turtle_pad_x, turtle_pad_y, 0.8, 0]
            pub_enable_auto_land.publish(Bool(True))
            publishGoals(goal_location)

        elif key == '6':
            goal_location = [-0.25, -0.25, 1.3, 1]
            publishGoals(goal_location)

        elif key == '7':
            goal_location = [-0.25, 0.25, 1.3, 1]
            publishGoals(goal_location)


if __name__ == "__main__":

    rospy.init_node('update_setpoint')

    # thread.start_new_thread(waitForKeyPress, ())

    bebop_ns = rospy.get_param("/bebop_ns")

    settings = termios.tcgetattr(sys.stdin)

    sub_pad = rospy.Subscriber(
        "vicon/turtle_pad/turtle_pad", TransformStamped, turtle_pad_callback)

    pub_x_goal = rospy.Publisher(
        bebop_ns + '/setpoint/x', Float64, queue_size=10)
    pub_y_goal = rospy.Publisher(
        bebop_ns + '/setpoint/y', Float64, queue_size=10)
    pub_z_goal = rospy.Publisher(
        bebop_ns + '/setpoint/z', Float64, queue_size=10)
    pub_yaw_goal = rospy.Publisher(
        bebop_ns + '/setpoint/yaw', Float64, queue_size=10)
    pub_enable_goal = rospy.Publisher(
        bebop_ns + '/setpoint/enable', Bool, queue_size=10)
    pub_takeoff = rospy.Publisher(bebop_ns + '/takeoff', Empty, queue_size=10)
    pub_land = rospy.Publisher(bebop_ns + '/land', Empty, queue_size=10)
    pub_emergency = rospy.Publisher(bebop_ns + '/reset', Empty, queue_size=10)

    pub_enable_auto_land = rospy.Publisher(bebop_ns + '/setpoint/enable_auto_land',
                                           Bool, queue_size=10)
    waitForKeyPress()

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
