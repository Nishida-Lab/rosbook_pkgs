#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random


class ISTATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_A'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state ISTATE')
        rospy.sleep(1)
        return 'to_A'


class BSTATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_C', 'to_D'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state BSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_C'

        return 'to_D'


class CSTATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_D', 'to_B', 'succeed'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state CSTATE')
        rand = random.random()
        rospy.sleep(1)

        if rand > 0.6:
            return 'to_D'
        elif rand > 0.3:
            return 'to_B'

        return 'succeed'


class DSTATE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_A', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state DSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_A'

        return 'failed'

class astate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_b', 'to_c'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state astate')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_b'

        return 'to_c'


class bstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_c', 'to_B'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state bstate')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_c'

        return 'to_B'


class cstate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_C'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state cstate')
        rospy.sleep(1)

        return 'to_C'


if __name__ == '__main__':
    rospy.init_node('action_average')
    sm = smach.StateMachine(outcomes=['succeed', 'failed'])

    with sm:
        smach.StateMachine.add('ISTATE', ISTATE(),
                               transitions={'to_A':'ASTATE'})
        ###
        sm_sub = smach.StateMachine(outcomes=['to_B', 'to_C'])
        with sm_sub:
            smach.StateMachine.add('astate', astate(),
                                   transitions={'to_b':'bstate',
                                                'to_c':'cstate'})
            smach.StateMachine.add('bstate', bstate(),
                                   transitions={'to_c':'cstate',
                                                'to_B':'to_B'})
            smach.StateMachine.add('cstate', cstate(),
                                   transitions={'to_C':'to_C'})
        smach.StateMachine.add('ASTATE', sm_sub,
                               transitions={'to_B':'BSTATE',
                                            'to_C':'CSTATE'})
        ###
        smach.StateMachine.add('BSTATE', BSTATE(),
                               transitions={'to_C':'CSTATE',
                                            'to_D':'DSTATE'})
        smach.StateMachine.add('CSTATE', CSTATE(),
                               transitions={'to_B':'BSTATE',
                                            'to_D':'DSTATE',
                                            'succeed':'succeed'})
        smach.StateMachine.add('DSTATE', DSTATE(),
                               transitions={'to_A':'ASTATE',
                                            'failed':'failed'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
