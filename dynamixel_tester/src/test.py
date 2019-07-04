#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.srv import DynamixelCommand

def dynamixel_tester():
    pub = rospy.Publisher('chatter', JointTrajectory, queue_size=10)
    rospy.init_node('dynamixel_tester', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    trajectory = JointTrajectory()

    trajectory.joint_names = ['End_link']
    trajectory.points = JointTrajectoryPoint()

    jtp = JointTrajectoryPoint()

    jtp.positions = [3.14]
    trajectory.points.append(jtp)
    pub.publish(trajectory)

def call_dymanixel_service(setpoint):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dyna_srv = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        resp = dyna_srv('', 1, 'Goal_Position', setpoint)
        print resp
        return resp.comm_result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
 #       dynamixel_tester()
        call_dymanixel_service(0)
        rospy.sleep(1)
        call_dymanixel_service(4095)
    except rospy.ROSInterruptException:
        pass