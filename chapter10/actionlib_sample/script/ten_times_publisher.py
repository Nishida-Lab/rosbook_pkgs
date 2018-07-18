#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import random

def ten_times_publish():
  pub = rospy.Publisher('number', Float32, queue_size=10)
  r = rospy.Rate(5)

  # invoke
  number = 1.0
  pub.publish(number)
  rospy.loginfo('published number[%d]: %f'%(0,number))
  r.sleep()

  for i in range(10):
    number = random.random()
    pub.publish(number)
    rospy.loginfo('published number[%d]: %f'%(i+1,number))
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('ten_times_publisher')
  ten_times_publish()
  rospy.spin()
