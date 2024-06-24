#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.19768, 1.77563, 0.3961),  # (0.17668, 1.75663, 0.4061),
                         (0.0, 0.0, 0.998, 0.067),  # 0.998, 0.061),
                         rospy.Time.now(),
                         "opti_corrected",
                         "base")
        rate.sleep()
