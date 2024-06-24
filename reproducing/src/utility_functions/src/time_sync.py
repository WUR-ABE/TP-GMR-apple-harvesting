#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
import message_filters


def cb_1(msg):
    global first_stamp, now
    for i, tf in enumerate(msg.transforms):
        if first_stamp is None:
            first_stamp = tf.header.stamp
        tf.header.stamp -= first_stamp
        tf.header.stamp += now
    pub_1.publish(msg)


def cb_2(msg):
    global first_stamp, now
    if first_stamp is None:
        first_stamp = msg.header.stamp
    msg.header.stamp -= first_stamp
    msg.header.stamp += now
    pub_2.publish(msg)


rospy.init_node('hoge')
first_stamp = None

rospy.sleep(1)
now = rospy.Time.now()

pub_2 = rospy.Publisher('/relaxed_ik/ee_pose_goals', PoseStamped, queue_size=1)

sub_2 = rospy.Subscriber('/pose_following/pose', PoseStamped, cb_2)

rospy.spin()
