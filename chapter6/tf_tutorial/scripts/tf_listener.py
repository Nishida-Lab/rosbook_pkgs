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
            (translation, rotation) = listener.lookupTransform('frame1', 'frame2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(1.0)
            continue
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rotation, axes='sxyz')

        rospy.loginfo("\n=== Got Transform ===\n"
                      " Translation\n"
                      " x : %f\n y : %f\n z : %f\n"
                      " Quaternion\n"
                      " x : %f\n y : %f\n z : %f\n w : %f\n"
                      " RPY\n"
                      " R : %f\n P : %f\n Y : %f",
                      translation[0], translation[1], translation[2],
                      rotation[0], rotation[1], rotation[2], rotation[3],
                      roll, pitch, yaw)
        r.sleep()
