#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    br = tf2_ros.TransformBroadcaster()

    r = rospy.Rate(1.0)
    
    while not rospy.is_shutdown():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'frame1'
        t.child_frame_id = 'frame2'
        
        translation = Vector3(0,0,1.0)
        t.transform.translation = translation
        
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0,'sxyz')
        rotation = Quaternion(*q)
        t.transform.rotation = rotation
        
        br.sendTransform(t)
        rospy.loginfo('Transform Published')
        r.sleep()
