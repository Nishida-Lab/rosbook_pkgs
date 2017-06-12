#!/usr/bin/env python
import rospy
import std_msgs.msg
import std_srvs.srv

if __name__ == '__main__':
    ## node initialization
    rospy.init_node('talker_node', anonymous=False)
    ## service call
    service_result = None
    rospy.loginfo('waiting service %s'%(rospy.resolve_name('servicename')))
    rospy.wait_for_service('servicename')
    try:
        srv_prox = rospy.ServiceProxy('servicename',
                                      std_srvs.srv.Trigger)
        res = srv_prox()
        if res.success:
            service_result = res.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    ## publisher
    pub = rospy.Publisher('topicname', std_msgs.msg.String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        str_msg = std_msgs.msg.String(data='string: ' + service_result)
        rospy.loginfo('%s publish %s'%(rospy.get_name(),str_msg.data))
        pub.publish(str_msg)
        rate.sleep()
