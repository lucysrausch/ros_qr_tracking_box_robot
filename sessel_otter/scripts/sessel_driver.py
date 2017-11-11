#!/usr/bin/env python2
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist, Vector3


pub = None

P_ROTATION   = 1
P_DISTANCE   = 1

SET_DISTANCE = 0.4 # distance in m

TAG_ID       = 5

def callback(data):
    global pub
    valid = False
    for i in data.markers:
        if (i.id == TAG_ID):
            valid = True
            rospy.loginfo("Marker %d detected!", i.id)
            speed = (i.pose.pose.position.z - SET_DISTANCE) * P_DISTANCE
            rotation = i.pose.pose.position.x * P_ROTATION
            pub.publish(Twist(linear = Vector3(rotation, 0, speed)))
    if not valid:
        pub.publish(Twist(linear = Vector3(0, 0, 0)))
        valid = False

def listener():
    global pub
    rospy.init_node('sessel_driver')
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    pub = rospy.Publisher('drive_command', Twist, queue_size = 1)
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        #rospy.loginfo("Otter!")
        r.sleep()

listener()
