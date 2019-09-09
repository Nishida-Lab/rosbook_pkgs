#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def callback(data):
    rospy .loginfo ("*")
    print 'buttons:["%s %s %s %s %s %s %s %s %s %s %s %s %s"]' % (data.buttons[0],data.buttons[1],data.buttons[2],data.buttons[3],data.buttons[4],data.buttons[5],data.buttons[6],data.buttons[7],data.buttons[8],data.buttons[9],data.buttons[10],data.buttons[11],data.buttons[12])
    print 'axes: ["%s %s %s %s %s %s"]' % (data.axes[0],data.axes[1],data.axes[2],data.axes[3],data.axes[4],data.axes[5])

    pan.publish(data.axes[0]*3.14/2.0)
    tilt.publish(data.axes[1]*3.14/2.0)

def listener():
    global pan
    pan = rospy.Publisher('/pan_joint/command',Float64)
    global tilt
    tilt = rospy.Publisher('/tilt_joint/command',Float64)

    rospy.init_node('listener', anonymous=True)
    rospy. Subscriber("joy" , Joy , callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
