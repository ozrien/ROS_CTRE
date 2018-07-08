#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped

import time


def callback(cmdVelocity):

    baseVelocity = TwistStamped()

    baseVelocity.twist = cmdVelocity

    now = rospy.get_rostime()
    baseVelocity.header.stamp.secs = now.secs
    baseVelocity.header.stamp.nsecs = now.nsecs
    baseVelocity.header.frame_id = "map"

    baseVelocityPub = rospy.Publisher('base_velocity', TwistStamped, queue_size=10)
    baseVelocityPub.publish(baseVelocity)


def cmd_vel_listener():

    rospy.Subscriber("/ctrerobot/diff_drive_controller/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_listener', anonymous=True)
    cmd_vel_listener()
