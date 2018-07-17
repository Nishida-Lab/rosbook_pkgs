#!/usr/bin/env python
import rospy
import actionlib
from actionlib import SimpleActionServer
from actionlib_tutorials.msg import AveragingAction
from std_msgs.msg import Float32

class AveragingSVR2(object):
  def __init__(self):
    self._action = SimpleActionServer('averaging',
            AveragingAction,
            auto_start = False)
    self._action.register_preempt_callback(self.preempt_cb)
    self._action.register_goal_callback(self.goal_cb)
    self.reset_numbers()
    rospy.Subscriber('number', Float32, self.execute_loop)
    self._action.start()

  def std_dev(self, lst):
    ave = sum(lst)/len(lst)
    return sum([x*x for x in lst])/len(lst) - ave**2

  def goal_cb(self):
    self._goal = self._action.accept_new_goal()
    rospy.loginfo('goal callback %s'%(self._goal))

  def preempt_cb(self):
    rospy.loginfo('preempt callback')
    self.reset_numbers()
    self._action.set_preempted(text='message for preempt')

  def reset_numbers(self):
    self._samples = []

  def execute_loop(self, msg):
    if (not self._action.is_active()):
      return
    self._samples.append(msg.data)
    feedback = AveragingAction().action_feedback.feedback
    feedback.sample = len(self._samples)
    feedback.data = msg.data
    feedback.mean = sum(self._samples)/len(self._samples)
    feedback.std_dev = self.std_dev(samples)
    self._action.publish_feedback(feedback)
    ## sending result
    if(len(self._samples) >= self._goal.samples):
      result = AveragingAction().action_result.result
      result.mean = sum(samples)/len(samples)
      result.std_dev = self.std_dev(samples)
      rospy.loginfo('result: %s'%(result))
      self.reset_numbers()
      if (result.mean > 0.5):
        self._action.set_succeeded(result = result, 
                text='message for succeeded')
      else:
        self._action.set_aborted(result = result,
                text='message for aborted')

if __name__ == '__main__':
  rospy.init_node('action_average')
  AveragingSVR2()
  rospy.spin()
