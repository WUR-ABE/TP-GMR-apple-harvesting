#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('goal_pose_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(125.0)
    while not rospy.is_shutdown():

        br.sendTransform((-0.11235, 0.29515, 0.4809),
                         (0.0, 0.0, -1.0, 0.0),
                         rospy.Time.now(),
                         "relaxed_ik",
                         "base")
        rate.sleep()
