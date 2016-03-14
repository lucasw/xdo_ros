#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher("string", String, queue_size=1)

while not rospy.is_shutdown():
    pub.publish(String("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwddddddddddddd"))
    rospy.Duration(0.2).sleep()
