#!/usr/bin/env python

import rospy
from dynamixel_workbench_msgs.msg import DynamixelStateList, DynamixelState
from std_msgs.msg import Float64
from math import pi
class EndLinkCurrentPosition():

    def __init__(self):
        rospy.init_node('end_link_current_position')
        self.pub = rospy.Publisher('/dynamix_encoder', Float64, queue_size=10)
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback)
        rospy.spin()

    def callback(self, msg):
        present_pos = msg.dynamixel_state[0].present_position
        offset = -1530
        corrected_present_pos = present_pos - offset
        if (corrected_present_pos < 0):
            corrected_present_pos += 4095
        if (corrected_present_pos > 4095):
            corrected_present_pos += -4095
        present_pos_rad = (float(corrected_present_pos)/4095)*(2*pi)-pi
        # rospy.loginfo('present_position: {}'.format (present_pos))
        rospy.loginfo('present_position: {}'.format (present_pos_rad))
        self.pub.publish(present_pos_rad)


if __name__ == "__main__":
        EndLinkCurrentPosition()
        
