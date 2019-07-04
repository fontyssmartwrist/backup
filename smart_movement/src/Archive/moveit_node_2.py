#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg 
from smart_movement.msg import smart_joint, codesys_joint
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random

class GeneratePathWithMoveIt():
    def __init__(self):
        # Initialize moveit_commandor
        moveit_commander.roscpp_initialize(sys.argv)
        # Create a node 
        rospy.init_node('moveit_command_centre',anonymous=True)
        # Setup control group of joints
        self.group = moveit_commander.MoveGroupCommander("smart_arm")
        # Read incoming data
        self.listen = rospy.Subscriber('/goal_smart_wrist', smart_joint, self.callback)

        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        #rospy.spin()

    def callback(self, data):
        # Store incoming data into angels of goal
        self.angles_of_goal = data
        # rospy.loginfo(self.angles_of_goal)
        # Make create a path to the goal
        self.go_to_joint_state(self.angles_of_goal)
       

    def go_to_joint_state(self):      
        # Set joint goals
        self.joint_goal = self.group.get_current_joint_values()
        self.joint_goal[0] = random.uniform(-pi,pi)
        self.joint_goal[1] = random.uniform(-pi,pi)
        self.joint_goal[2] = 0
        self.joint_goal[3] = -self.joint_goal[1]
        self.joint_goal[4] = random.uniform(-pi,pi)

        
        """
        self.joint_goal[0] = theta.theta0
        self.joint_goal[1] = theta.theta1
        self.joint_goal[2] = theta.theta2
        self.joint_goal[3] = theta.theta3
        """

        self.plan_of_wrist = self.group.plan()
        self.send_goal()
        
        # Execute moving to the joint until movement is reached
        self.group.go(self.joint_goal, wait=True)
        # Stop movement to ensure that there is no residual movement
        self.group.stop()

    def send_goal(self):
        # Get information of the path
        print " "
        print(self.plan_of_wrist.joint_trajectory.points[0])
        self.servo_waypoints = codesys_joint()

        #for nb_of_element in range(0,len(self.plan_of_wrist.joint_trajectory.points)):
        #    self.moveit_waypoint_theta0 = self.plan_of_wrist.joint_trajectory.points[nb_of_element].positions[0]
    #        self.moveit_waypoint_theta1 = self.plan_of_wrist.joint_trajectory.points[nb_of_element].positions[1]
    #        self.moveit_waypoint_theta2 = self.plan_of_wrist.joint_trajectory.points[nb_of_element].positions[2]
    #        self.moveit_waypoint_theta3 = self.plan_of_wrist.joint_trajectory.points[nb_of_element].positions[3]
    #        self.servo_waypoint.theta0[nb_of_element] = int((self.moveit_waypoint_theta0 / pi) * 100000)
    #        self.servo_waypoint.theta1[nb_of_element] = int((self.moveit_waypoint_theta1 / pi) * 100000)
    #        self.servo_waypoint.theta2[nb_of_element] = int((self.moveit_waypoint_theta2 / pi) * 100000)
    #        self.servo_waypoint.theta3[nb_of_element] = int((self.moveit_waypoint_theta3 / pi) * 100000)
        
        #self.give_waypoints.publish(self.servo_waypoints)
            
              
        

    


if __name__ == '__main__':
    try:
        # Start the main class
        a=GeneratePathWithMoveIt()
        a.go_to_joint_state()
        a.go_to_joint_state()
        a.go_to_joint_state()
        a.go_to_joint_state()
        a.go_to_joint_state()

    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass