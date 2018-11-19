#!/usr/bin/env python
import rospy, actionlib, math
from geometry_msgs.msg import Twist
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_controllers_msgs.msg import Pr2GripperCommandAction
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal
# smach
import smach
import smach_ros
from smach import CBState
from smach_ros import ServiceState
from smach_ros import SimpleActionState


class PR2move(object):
    def __init__(self):
        self.base_pub = rospy.Publisher('/base_controller/command',
                                        Twist, queue_size=10)
        self.larm_act = actionlib.SimpleActionClient(
            '/l_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.rarm_act = actionlib.SimpleActionClient(
            '/r_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.torso_act = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.head_act = actionlib.SimpleActionClient(
            '/head_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.head_act.wait_for_server()

    def move_base(self, msg):
        rate = rospy.Rate(4)
        cntr = 0
        direction = msg * 4

        while not rospy.is_shutdown():
            cntr += 1
            msg = Twist()
            msg.linear.x = direction
            self.base_pub.publish(msg)

            if cntr == 25:
                break

            rate.sleep()

    def pr2_pose(self, pose_str):
        self.send_pose(['torso_lift_joint'], [0.15 * 180 / math.pi], self.torso_act)
        self.send_pose(['head_pan_joint', 'head_tilt_joint'], [0, 40], self.head_act)
        self.send_pose(['r_shoulder_pan_joint', 'r_shoulder_lift_joint',
                        'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
                        'r_forearm_roll_joint', 'r_wrist_flex_joint',
                        'r_wrist_roll_joint'],
                       [-75, 50, -110, -110, 20, -10, -10], self.rarm_act)

        pose = [75, 50, 110, -110, -20, -10, -10]  # reset

        if pose_str == 'pregrasp':
            pose = [23.1, 64.8, 37.4, -97.0, -158.7, -32.3, 146.4]
        elif pose_str == 'grasp':
            pose = [15.0, 50.5, 30.5, -71.8, -155.6, -22.1, 138.1]
        elif pose_str == 'pickup':
            pose = [18.1, 43.5, 36.7, -78.0, -164.8, -33.1, 141.2]

        self.send_pose(['l_shoulder_pan_joint', 'l_shoulder_lift_joint',
                        'l_upper_arm_roll_joint', 'l_elbow_flex_joint',
                        'l_forearm_roll_joint', 'l_wrist_flex_joint',
                        'l_wrist_roll_joint'], pose, self.larm_act)

        for act in [self.head_act, self.rarm_act,
                    self.larm_act, self.head_act]:
            res = act.wait_for_result()

        if res:
            rospy.loginfo('get result: %s' % act.get_result())
            rospy.loginfo('result state: %s' % actionlib.GoalStatus.to_string(act.get_state()))
        else:
            rospy.loginfo('fail action %s' % act)

    def send_pose(self, names, angles, act, duration=5):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [x * math.pi / 180 for x in angles]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [ point ]
        goal.trajectory.header.stamp = rospy.Time.now()
        act.send_goal(goal)


def move_cb(userdata, _pr2_move, direction):
    pr2move.move_base(direction)
    return 'success'


def pose_cb(userdata, _pr2_move, pose):
    pr2move.pr2_pose(pose)
    return 'success'


if __name__ == '__main__':
    rospy.init_node('pr2move')
    pr2move = PR2move()
    sm = smach.StateMachine(outcomes=['success'])

    with sm:
        init_sub = smach.StateMachine(outcomes=['success'])
        with init_sub:
            gripper_open  = Pr2GripperCommandGoal()
            gripper_open.command.position = 0.025
            gripper_open.command.max_effort = 2000
            smach.StateMachine.add('Open_gripper',
                                   SimpleActionState('/l_gripper_controller/gripper_action',
                                                     Pr2GripperCommandAction,
                                                     goal=gripper_open),
                                   transitions={'succeeded': 'Init_pose',
                                                'preempted': 'Init_pose',
                                                'aborted': 'Open_gripper'})
            smach.StateMachine.add('Init_pose',
                                   CBState(pose_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, 'reset']),
                                   {'success': 'Move_forward'})
            smach.StateMachine.add('Move_forward',
                                   CBState(move_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, 1]),
                                   {'success': 'success'})

        smach.StateMachine.add('MOVE_FORWARD', init_sub,
                               transitions={'success': 'GRASP'})

        grasp_sub = smach.StateMachine(outcomes=['success'])

        with grasp_sub:
            smach.StateMachine.add('Move_arm',
                                   CBState(pose_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, 'pregrasp']),
                                   {'success': 'Move_arm_grasp'})
            smach.StateMachine.add('Move_arm_grasp',
                                   CBState(pose_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, 'grasp']),
                                   {'success': 'Close_gripper'})
            gripper_close  = Pr2GripperCommandGoal()
            gripper_close.command.position = 0.00
            gripper_close.command.max_effort = 20
            smach.StateMachine.add('Close_gripper',
                                   SimpleActionState('/l_gripper_controller/gripper_action',
                                                     Pr2GripperCommandAction,
                                                     goal=gripper_close),
                                   transitions={'succeeded': 'success',
                                                'preempted': 'success',
                                                'aborted': 'success'})

        smach.StateMachine.add('GRASP', grasp_sub,
                               transitions={'success': 'FINISH'})

        finish_sub = smach.StateMachine(outcomes=['success'])

        with finish_sub:
            smach.StateMachine.add('Pickup_pose',
                                   CBState(pose_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, 'pickup']),
                                   {'success': 'Move_backward'})
            smach.StateMachine.add('Move_backward',
                                   CBState(move_cb,
                                           outcomes=['success'],
                                           cb_args=[pr2move, -1]),
                                   {'success': 'success'})

        smach.StateMachine.add('FINISH', finish_sub,
                               transitions={'success': 'success'})

        sis = smach_ros.IntrospectionServer('server_name', sm, '/ROOT')
        sis.start()
        rospy.sleep(1)
        result = sm.execute()
        rospy.loginfo('result: %s' % result)
        rospy.spin()
        sis.stop()
