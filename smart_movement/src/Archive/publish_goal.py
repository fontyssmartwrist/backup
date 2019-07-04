#!/usr/bin/env python

import rospy
from smart_movement.msg import smart_joint
import random
from math import pi

def talker():
    pub = rospy.Publisher('/goal_smart_wrist', smart_joint, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    while not rospy.is_shutdown():
        joint_goal = smart_joint()
        joint_goal.theta0 = random.uniform(-pi,pi)
        joint_goal.theta1 = random.uniform(-pi,pi)
        joint_goal.theta2 = random.uniform(-pi,pi)
        joint_goal.theta3 = random.uniform(-pi,pi)
        joint_goal.theta4 = random.uniform(-pi,pi)
        rospy.loginfo("message send")
        rospy.loginfo(joint_goal)
        pub.publish(joint_goal)
        rospy.sleep(4)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
