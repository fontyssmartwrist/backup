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
from moveit_commander.conversions import pose_to_list

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
        # Publish the codesys waypoints to comunication node
        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        # Keep the script running
        rospy.spin()

    def callback(self, data):
        # Store incoming data into angels of goal
        self.angles_of_goal = data
        # Make create a path to the goal
        self.go_to_joint_state(self.angles_of_goal)
       

    def go_to_joint_state(self, theta):      
        # Set joint goal type
        self.joint_goal = self.group.get_current_joint_values()
        # Set each joint to the correct value
        self.joint_goal[0] = theta.theta0
        self.joint_goal[1] = theta.theta1
        self.joint_goal[2] = theta.theta2
        self.joint_goal[3] = theta.theta3
        self.joint_goal[4] = theta.theta4
        # Make a plan to move to the next goal
        self.plan_of_wrist = self.group.plan(self.joint_goal)

        # Send waypoints of the goal
        self.send_goal()
        
        # Execute moving to the joint until movement is reached
        self.plan = self.group.go(self.joint_goal, wait=True)
        print self.plan

        # Stop movement to ensure that there is no residual movement
        self.group.stop()

    def send_goal(self):
        # Get information of the path
        self.amount_of_waypoints = len(self.plan_of_wrist.joint_trajectory.points)
        # Set variable to correct message type
        self.servo_waypoints = codesys_joint()
        # Set the amount of waypoints
        self.servo_waypoints.waypoints = self.amount_of_waypoints
        # Create empty lists to store the data
        self.servo_waypoints.theta0 = [0] * self.amount_of_waypoints
        self.servo_waypoints.theta1 = [0] * self.amount_of_waypoints
        self.servo_waypoints.theta2 = [0] * self.amount_of_waypoints
        self.servo_waypoints.theta3 = [0] * self.amount_of_waypoints
        # Loop for storing the trajectory in servo angles used in codesys, every cycle stores one waypoint
        for nb_of_waypoints in range(0, self.amount_of_waypoints):
            # Store the joint values in temporary variables for clearity
            self.moveit_waypoint_theta0 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[0]
            self.moveit_waypoint_theta1 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[1]
            self.moveit_waypoint_theta2 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[2]
            self.moveit_waypoint_theta3 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[3]
            # Translate the float64 points between -pi and pi to integer values used by the codesys 
            # and store them 
            self.servo_waypoints.theta0[nb_of_waypoints] = int((self.moveit_waypoint_theta0 / pi) * 100000)
            self.servo_waypoints.theta1[nb_of_waypoints] = int((self.moveit_waypoint_theta1 / pi) * 100000)
            self.servo_waypoints.theta2[nb_of_waypoints] = int((self.moveit_waypoint_theta2 / pi) * 100000)
            self.servo_waypoints.theta3[nb_of_waypoints] = int((self.moveit_waypoint_theta3 / pi) * 100000)
        # Print servo_waypoints to check if message was correct
        print self.servo_waypoints
        # Publish list of waypoints to /codesys_waypoints
        self.give_waypoints.publish(self.servo_waypoints)
            


if __name__ == '__main__':
    try:
        # Start the main class
        GeneratePathWithMoveIt()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass