#!/usr/bin/env python
import rospy
import actionlib
from actionlib import SimpleActionServer
from actionlib_tutorials.msg import AveragingAction
import random

class AveragingSVR(object):
  def __init__(self):
    self._action = SimpleActionServer('averaging',
            AveragingAction,
            execute_cb = self.execute_cb,
            auto_start = False)
    self._action.register_preempt_callback(self.preempt_cb)
    self._action.start()

  def std_dev(self, lst):
    ave = sum(lst)/len(lst)
    return sum([x*x for x in lst])/len(lst) - ave**2

  def preempt_cb(self):
    rospy.loginfo('preempt callback')
    self._action.set_preempted(text='message for preempt')

  def execute_cb(self, goal):
    rospy.loginfo('execute callback: %s'%(goal))
    feedback = AveragingAction().action_feedback.feedback
    result = AverangAction().action_result.result
    ## execute loop
    rate = rospy.Rate(1/(0.01 + 0.99*random.random()))
    samples= []
    for i in range(goal.samples):
      sample = random.random()
      samples.append(sample)
      feedback.sample = i
      feedback.data = sample
      feedback.mean = sum(samples)/len(samples)
      feedback.std_dev = self.std_dev(samples)
      self._action.publish_feedback(feedback)
      rate.sleep()
    if(not self._action.is_active()):
      rospy.loginfo('not active')
      return
    ## sending result
    result.mean = sum(samples)/len(samples)
    result.std_dev = self.std_dev(samples)
    rospy.loginfo('result: %s'%(result))
    if (result.mean > 0.5):
      self._action.set_succeeded(result = result, text='message for succeeded')
    else:
      self._action.set_aborted(result = result, text='message for aborted')

if __name__ == '__main__':
  rospy.init_node('action_average')
  AveragingSVR()
  rospy.spin()
