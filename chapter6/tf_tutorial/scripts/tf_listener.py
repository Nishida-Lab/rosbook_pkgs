#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    r = rospy.Rate(1.0)
    
    while not rospy.is_shutdown():
        try:
            (origin, rotation) = listener.lookupTransform('frame2', 'frame1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(1.0)
            continue

        rospy.loginfo("\n=== Got Transform ===\n"
                      " x : %f\n y : %f\n z : %f\n"
                      " x : %f\n y : %f\n z : %f\n w : %f",
                      origin[0], origin[1], origin[2],
                      rotation[0], rotation[1], rotation[2], rotation[3])
        r.sleep()
