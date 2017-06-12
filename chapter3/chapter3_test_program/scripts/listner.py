#!/usr/bin/env python
import rospy
import std_msgs.msg
import std_srvs.srv

def callback(data):
    rospy.loginfo(rospy.get_name() + ' / I heard %s', data.data)

def service_callback(req):
    tm = rospy.get_time()
    param_str = None
    if rospy.has_param('~private_param_name'):
        param_str = rospy.get_param('~private_param_name')
    rospy.logwarn('%s_%s: service called for %s'%(rospy.get_name(),tm,param_str))
    return std_srvs.srv.TriggerResponse (success = True,
             message = '%s_%s: service returned %s'%(rospy.get_name(),tm,param_str))

if __name__ == '__main__':
    ## node initalization
    rospy.init_node('listener_node', anonymous=False)
    ## parameter
    if rospy.has_param('/global_param_name'):
        gparam = rospy.get_param('/global_param_name')
        rospy.loginfo('global_param: %s'%gparam)
    if rospy.has_param('param_name'):
        nparam = rospy.get_param('param_name')
        rospy.loginfo('normal_param: %s'%nparam)
    if rospy.has_param('~private_param_name'):
        lparam = rospy.get_param('~private_param_name')
        rospy.loginfo('private_param: %s'%lparam)
    ## subscriber
    rospy.Subscriber('topicname', std_msgs.msg.String, callback)
    ## service
    rospy.Service('servicename', std_srvs.srv.Trigger, service_callback)
    ##
    rospy.spin()
