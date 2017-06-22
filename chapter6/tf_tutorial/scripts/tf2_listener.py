#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf_conversions

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(1.0)
    
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform('frame1', 'frame2', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('LookupTransform Error !')
            rospy.sleep(1.0)
            continue

        translation = (transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z)
        
        rotation = (transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w)
        
        (roll, pitch, yaw) = tf_conversions.transformations.euler_from_quaternion(rotation, axes='sxyz')

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
