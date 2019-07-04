#!/usr/bin/env python

import rospy
from smart_movement.msg import CurrentJointValues
import random
from math import pi
from sensor_msgs.msg import JointState

class ImitateCodesysValues():

    def __init__(self):
        rospy.init_node('codesys', anonymous=True)
        self.sub = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.rewrite_values)
        self.pub = rospy.Publisher('/current_joint_values', CurrentJointValues, queue_size=10)
        print 1
        self.wait()
        self.talker()
        
    def wait(self):
        rospy.sleep(5)

    def talker(self):
        while not rospy.is_shutdown():
            rospy.loginfo("message send")
            rospy.loginfo(self.scrambled_value)
            self.pub.publish(self.scrambled_value)
            rospy.sleep(1)
        


    def rewrite_values(self,data):
        self.wrong_value = data
        self.scrambled_value = CurrentJointValues()
        self.scrambled_value.theta0 = int((self.wrong_value.position[0] / pi) * 100000)
        self.scrambled_value.theta1 = int((self.wrong_value.position[1] / pi) * 100000)
        self.scrambled_value.theta2 = int((self.wrong_value.position[2] / pi) * 100000)
        self.scrambled_value.theta3 = int((self.wrong_value.position[3] / pi) * 100000)
        self.scrambled_value.theta0 = self.scrambled_value.theta0 + int(random.uniform(-20,20))
        self.scrambled_value.theta1 = self.scrambled_value.theta1 + int(random.uniform(-20,20))
        self.scrambled_value.theta2 = self.scrambled_value.theta2 + int(random.uniform(-20,20))
        self.scrambled_value.theta3 = self.scrambled_value.theta3 + int(random.uniform(-20,20))
        
        
if __name__ == '__main__':
    try:
        ImitateCodesysValues()
    except rospy.ROSInterruptException:
        pass
