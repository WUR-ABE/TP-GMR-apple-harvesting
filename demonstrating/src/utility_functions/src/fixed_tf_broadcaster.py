#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.7534, 0.5602, 0.6736),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "opti_corrected",
                         "world")
        rate.sleep()
