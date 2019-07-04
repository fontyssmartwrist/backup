#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import Float64
import math

class EndLinkController():
    def __init__(self):
        self.goal_sub = rospy.Subscriber("/end_link_controller/goal", Float64, self.goal_callback)
        rospy.loginfo("Waiting for dynamixel Service")
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        rospy.loginfo("Found service")
	    
        self.store = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        
        rospy.spin()
    
    
    # self.dynamixel_data(self, req):

    # rospy.loginfo("Waiting for the data from moveit")
    def goal_callback(self, goal_angle_rad):

        if goal_angle_rad.data >= -math.pi and goal_angle_rad.data <= math.pi:
            rospy.loginfo(
                "Setting end_link to goal position {}".format(goal_angle_rad.data))
            self.store('', 1, 'Goal_Position',
                               self.angle_to_position(goal_angle_rad.data))

            

        else:
            rospy.loginfo("Goal out of range: -pi to +pi")

    def angle_to_position(self, angle_rad):
        setpoint = (angle_rad + math.pi) * (4095 / (2*math.pi))
        offset = -1530
        corrected_setpoint = setpoint + offset
        if (corrected_setpoint < 0):
            corrected_setpoint += 4095
        if (corrected_setpoint > 4095):
            corrected_setpoint += -4095
        rospy.loginfo(corrected_setpoint)
        return corrected_setpoint
	

        


if __name__ == '__main__':
    try:
        rospy.init_node('end_link_controller')
        controller = EndLinkController()
    except rospy.ROSInterruptException:
        pass
