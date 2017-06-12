#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty, String, Float64
# old geofence
geofence_x = [-2, 0.1]
geofence_y = [-.1, 2.6]
# new geofence w origin at turtle_bot dock corner
geofence_x = [-2.45, 0.5]
geofence_y = [-.4, 1.94]

geofence_z = 1.5


def callback(data):

    killDrone = False

    translations = data.transform.translation
    quat = data.transform.rotation
    vicon_x = translations.x
    vicon_y = translations.y
    vicon_z = translations.z

    if vicon_x < geofence_x[0] or vicon_x > geofence_x[1] or vicon_y < geofence_y[0] or vicon_y > geofence_y[1]:
        killDrone = True
        empty = Empty()
        pub_emergency.publish(empty)
        print('**** Broke geofence position = ' +
              str(vicon_x) + ',' + str(vicon_y) + ' ****')

    if vicon_z > geofence_z:
        pub_land.publish(Empty())


if __name__ == "__main__":

    rospy.init_node('bebop_geofence')

    bebop_ns = rospy.get_param("/bebop_ns")

    pub_emergency = rospy.Publisher(bebop_ns + '/reset', Empty, queue_size=10)
    pub_land = rospy.Publisher(bebop_ns + '/land', Empty, queue_size=10)

    sub = rospy.Subscriber("vicon/bebop1/bebop1", TransformStamped, callback)

    rospy.spin()
