#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg 
import random
from math import pi
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint, CurrentJointValues, smart_joint
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float64

class GeneratePathWithMoveIt():
    def __init__(self):
        # Initialize moveit_commandor
        moveit_commander.roscpp_initialize(sys.argv)
        # Create a node 
        rospy.init_node('moveit_command_centre')
        # Setup control group of joints
        self.group = moveit_commander.MoveGroupCommander("smart_arm")
        # Read incoming data from laser element in the web server
        self.goal_listen = rospy.Subscriber('/web_laser', geometry_msgs.msg.Point, self.translate_to_3d)
        # Send goal to end link which is the dynamixel motor
        self.send_dynamixel = rospy.Publisher('/end_link_controller/goal', Float64, queue_size=10)
        # Get joint goal from the web
        self.web_goal = rospy.Subscriber('/web_goal', Float32MultiArray, self.go_to_joint_state)
        # Get the leveling goal from the web control
        self.web_orientation_goal = rospy.Subscriber('/web_orientation_goal', geometry_msgs.msg.Quaternion, self.go_to_orientation)
        # Get the real joint values from codesys
        self.real_listen = rospy.Subscriber('/current_joint_values', CurrentJointValues, self.set_current_pos)
        # Publish the codesys waypoints to comunication node
        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        # Get the current joint posistions
        self.current_joint_info()
        # Keep the script running
        rospy.spin()

    def get_the_joints(self, req):
        rospy.loginfo(req.statusreq)
        current_pos = self.group.get_current_joint_values()
        return RequestStatusResponse(str(current_pos))

    def set_current_pos(self, joint_data):
        # Build the current position message
        self.current_pos = JointState()
        self.current_pos.header.seq = 1
        self.current_pos.header.stamp = rospy.Time.now()
        self.current_pos.header.frame_id = "/world"
        self.current_pos.name = ["joint1", "joint2", "joint3", "joint5"]
        self.current_pos.position = [joint_data.theta2, joint_data.theta3, dynamix_data.data, -1.735]
        rospy.loginfo("Joint values were set manually")
        # Publish the current position to the simulation
        self.give_current_pose.publish(self.current_pos)

    def translate_to_3d (self, xy):
        rospy.loginfo("Going to coordinate: {}".format(xy))
        x = 0.18 - xy.x * 0.04
        z = 0.60 - xy.y * 0.04
        xyz = [x, -1.735, z]
        self.go_to_position(xyz)

    def go_to_position(self, xyz):
        # Log info about the goal
        rospy.loginfo("Going to postition: {}".format(xyz))
        # Set the position target 
        self.group.set_position_target(xyz, "link5")
        # Command moveit to calculate a plan with the set goal and store it
        self.plan_of_wrist = self.group.plan()
        # rospy.loginfo(self.plan_of_wrist)
        # Send the path to the codesys software
        self.send_goal()
        # Call the MoveIt! commander to compute the plan and execute it.
        go = self.group.go(wait=False) # The program holds until pose is reached

    def go_to_orientation(self, quat):
        # Log info about the goal
        rospy.loginfo("Going to orientation: {}".format(quat))
        # Set the orionitation target 
        quatgoal = [quat.x, quat.y, quat.z, quat.w]
        self.group.set_orientation_target(quatgoal)
        # Command moveit to calculate a plan with the set goal and store it
        self.plan_of_wrist = self.group.plan()
        # Send the path to the codesys software
        self.send_goal()
        # Call the MoveIt! commander to compute the plan and execute it.
        self.group.go(wait=False) # The program holds until pose is reached

    def go_to_joint_state(self, theta):      
        # Set joint goal message type
        self.joint_goal = self.group.get_current_joint_values()
        # Set each joint to the correct value
        self.joint_goal[0] = theta.data[0]
        self.joint_goal[1] = theta.data[1]
        self.joint_goal[2] = theta.data[2]
        self.joint_goal[3] = 0.07
        # Make a plan to move to the next goal
        self.plan_of_wrist = self.group.plan(self.joint_goal)
        # Send waypoints of the goal
        self.send_goal()
        # Execute moving to the joint until movement is reached
        self.group.go(self.joint_goal, wait=False)

    def send_goal(self):
        # Get information of the path
        amount_of_waypoints = len(self.plan_of_wrist.joint_trajectory.points)
        # Set variable to correct message type
        servo_waypoints = codesys_joint()
        # Set the amount of waypoints
        servo_waypoints.waypoints = amount_of_waypoints
        # Create empty lists to store the data
        servo_waypoints.theta0 = [0] * amount_of_waypoints
        servo_waypoints.theta1 = [0] * amount_of_waypoints
        servo_waypoints.theta2 = [0] * amount_of_waypoints
        servo_waypoints.theta3 = [0] * amount_of_waypoints
        last_link_waypoint = 0.0

        # Loop for storing the trajectory in servo angles used in codesys, every cycle stores one waypoint
        for nb_of_waypoints in range(0, amount_of_waypoints):
            # Store the joint values in temporary variables for clearity
            # moveit_waypoint_theta0 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[0]
            # moveit_waypoint_theta1 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[1] 
            moveit_waypoint_theta0 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[0]
            moveit_waypoint_theta1 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[1]
            moveit_waypoint_theta2 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[2] 
            # Translate the float64 points between -pi and pi to integer values used by the codesys 
            # and store them 
            servo_waypoints.theta0[nb_of_waypoints] = int(0)
            servo_waypoints.theta1[nb_of_waypoints] = int(0)
            servo_waypoints.theta2[nb_of_waypoints] = int((moveit_waypoint_theta0 / pi) * 3140)
            servo_waypoints.theta3[nb_of_waypoints] = int((moveit_waypoint_theta1 / pi) * 3140)
        # Pservo_waypoints to check if message was correct
        print servo_waypoints
        # Wait for the subscriber 
        # while (self.give_waypoints.get_num_connections() == 0):
        #     pass 
        if (amount_of_waypoints != 0):
            # Publish list of waypoints to /codesys_waypoints
            self.give_waypoints.publish(servo_waypoints)
            rospy.loginfo("Waypoints sent to the server")
            # Get last link last angle and publish it to the dynamixel
            last_link_waypoint = self.plan_of_wrist.joint_trajectory.points[amount_of_waypoints-1].positions[2]
            self.send_dynamixel.publish(last_link_waypoint)
            rospy.loginfo("Goal sent to dynamixel: {}".format(last_link_waypoint))
        else:
            rospy.logwarn("O waypoints, no path available")

    def current_joint_info(self):
        rospy.loginfo('====================')
        safe = self.group.get_current_joint_values()
        rospy.loginfo(safe)

if __name__ == '__main__':
    try:
        # Start the main class
        GeneratePathWithMoveIt()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass