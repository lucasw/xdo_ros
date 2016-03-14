#!/usr/bin/env python

import rospy
from opencv_apps.msg import Point2D
from std_msgs.msg import String

rospy.init_node('loop_and_send_keys')
down_pub = rospy.Publisher("keys_down", String, queue_size=1)
up_pub = rospy.Publisher("keys_up", String, queue_size=1)
mouse_rel_pub = rospy.Publisher("mouse_rel", Point2D, queue_size=1)

down_pub.publish(String("w"))
while not rospy.is_shutdown():
    # mouse_rel_pub.publish(Point2D(200,0))
    # rospy.sleep(2.0)
    # continue

    down_pub.publish(String("w"))
    rospy.sleep(2.0)
    up_pub.publish(String("w"))

    down_pub.publish(String("d"))
    rospy.sleep(2)
    up_pub.publish(String("d"))

    down_pub.publish(String("w"))
    rospy.sleep(2.0)
    up_pub.publish(String("w"))

    down_pub.publish(String("a"))
    rospy.sleep(2)
    up_pub.publish(String("a"))

    mouse_rel_pub.publish(Point2D(200,0))
