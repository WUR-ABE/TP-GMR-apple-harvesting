#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tcp_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.707, -0.707, 0.0),
                         rospy.Time.now(),
                         "tool_ik",
                         "tool0")
        rate.sleep()
