#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('goal_pose_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.0, -0.06, -0.03),
                         (0.0, 0.766, -0.643, 0.0),
                         rospy.Time.now(),
                         "goal_ik",
                         "Experiment_Nick")
        rate.sleep()
