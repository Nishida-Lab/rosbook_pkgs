#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    br = tf.TransformBroadcaster()

    r = rospy.Rate(1.0)
    
    while not rospy.is_shutdown():
        translation = (0, 0, 1.0)
        rotation = tf.transformations.quaternion_from_euler(0,0,0, axes='sxyz')
        br.sendTransform(translation, rotation,
                         rospy.Time.now(),'frame2', 'frame1')
        rospy.loginfo('Transform Published')
        r.sleep()
