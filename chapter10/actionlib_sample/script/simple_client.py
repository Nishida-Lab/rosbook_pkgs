#!/usr/bin/env python
import rospy
import actionlib
from actionlib_tutorials.msg import AveragingAction

def feedback_callback(msg):
  rospy.loginfo('feedback %s'%(msg))

if __name__ == '__main__':
  rospy.init_node('call_action') ## initialize node
  ## creating simple action client
  act = actionlib.SimpleActionClient('averaging',
          AveragingAction)
  rospy.loginfo('waiting server') ## waiting server
  act.wait_for_server(rospy.Duration(5))
  rospy.loginfo('send goal')
  goal = AveragingAction().action_goal.goal
  goal.samples = 10 ## number of samples
  act.send_goal(goal, feedback_cb = feedback_callback)
  rospy.loginfo('waiting result')
  ret = act.wait_for_result(rospy.Duration(100))
  if(ret):
    rospy.loginfo('get result: %s'%(act.get_result()))
    rospy.loginfo('result state: %s'%(actionlib.GoalStatus.to_string(act.get_state())))
    rospy.loginfo('result text: ' + act.get_goal_status_text())
  else:
    rospy.loginfo('cancel')
    act.cancel_goal()
    # act.cancel_all_goals() ## cancel including another goal
  rospy.spin()
