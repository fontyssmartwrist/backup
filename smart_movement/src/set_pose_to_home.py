#!/usr/bin/env python

import rospy
from smart_movement.msg import CurrentJointValues
import random
from math import pi

def talker():
    pub = rospy.Publisher('/current_joint_values', CurrentJointValues, queue_size=10)
    rospy.init_node('reseter')
    joint_goal = CurrentJointValues()
    joint_goal.theta0 = 0.0
    joint_goal.theta1 = 0.0
    joint_goal.theta2 = 0.0
    joint_goal.theta3 = 0.0
    rospy.loginfo("reset send")
    rospy.loginfo(joint_goal)
    pub.publish(joint_goal)
    rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
