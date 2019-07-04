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
from std_msgs.msg import Float32MultiArray
from datetime import datetime, timedelta
import time 

class GeneratePathWithMoveIt():
    def __init__(self):
        # Initialize moveit_commandor
        moveit_commander.roscpp_initialize(sys.argv)
        # Create a node 
        rospy.init_node('moveit_command_centre',anonymous=True)
        # Setup control group of joints
        self.group = moveit_commander.MoveGroupCommander("smart_arm")
        # Read incoming data
        self.goal_listen = rospy.Subscriber('/goal_smart_wrist', geometry_msgs.msg.Pose, self.callback)
        # Publish the codesys waypoints to comunication node
        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        # Get the real joint values from codesys
        self.real_listen = rospy.Subscriber('/current_joint_values', CurrentJointValues, self.set_current_pos)
        # Send current pose to demo
        self.give_current_pose = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        # Get joint goal from the web
        # self.web_goal = rospy.Subscriber('/web_goal', Float32MultiArray, self.go_to_joint_state)
        # Move in a straight line demo
        ## self.straight_line()
        # Prove move to position is not working
        # self.go_to_pos()
        # Keep the script running
        rospy.spin()

    def callback(self, goal_data):
        print ("==============================================================================")
        # Store incoming data into angels of goal
        self.send_flag = True
        self.data_of_goal = goal_data
        # Make create a path to the goal
        self.go_to_pose_goal(self.data_of_goal)

    def set_current_pos(self, joint_data):
        # Build the current position message
        self.current_pos = JointState()
        self.current_pos.header.seq = 99999
        self.current_pos.header.stamp = rospy.Time.now()
        self.current_pos.header.frame_id = "/world"
        self.current_pos.name = ["end_link", "joint0", "joint1", "joint3", "joint4"]
        self.current_pos.position = [0.0, joint_data.theta0, joint_data.theta1, joint_data.theta2, joint_data.theta3]
        print self.current_pos
        # Publish the current position to the simulation
        self.give_current_pose.publish(self.current_pos)

    # def go_to_pos(self):
    #     # rospy.loginfo("Going to postition(only give xyz)")
    #     # xyz= [0.0, 0.0, 1.3229]
    #     # rospy.loginfo(xyz)
    #     # self.group.set_position_target(xyz)
    #     # self.plan_of_wrist = self.group.plan()
    #     # self.send_goal()
    #     # self.group.go(wait=True)
    #     rospy.loginfo("Going to pose(position and orientation)")
    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.position.x = 0.0
    #     pose_goal.position.y = 0.0
    #     pose_goal.position.z = 1.3229
    #     pose_goal.orientation.x = 0.0
    #     pose_goal.orientation.y = 0.0
    #     pose_goal.orientation.z = 0.0
    #     pose_goal.orientation.w = 1.0
    #     rospy.loginfo(pose_goal)
    #     self.group.set_pose_target(pose_goal)
    #     self.plan_of_wrist = self.group.plan()
    #     print ("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    #     print (self.plan_of_wrist)
    #     print ("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    #     self.send_goal()
    #     self.group.go(wait=True)

    def go_to_pose_goal(self, pose_goal):
        # Set the pose target with the send pose
        self.group.set_pose_target(pose_goal)
        # Extract the plan with the set pose
        self.plan_of_wrist = self.group.plan()
        #print "Joint endgoal is:", self.group.get_current_joint_values()
        #rospy.loginfo(self.plan_of_wrist)
        # Send the path to the codesys software
        self.send_goal()
        # Now, we call the planner to compute the plan and execute it.
        #self.plan = self.group.go(wait=False)
        # Calling `stop()` ensures that there is no residual movement
        #self.group.stop()
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()


    # def go_to_joint_state(self, theta):      
    #     # Set joint goal message type
    #     self.joint_goal = self.group.get_current_joint_values()
    #     # Set each joint to the correct value
    #     self.joint_goal[0] = theta.data[0]
    #     self.joint_goal[1] = theta.data[1]
    #     self.joint_goal[2] = theta.data[2]
    #     self.joint_goal[3] = theta.data[3]
    #     self.joint_goal[4] = 0.0
    #     # Make a plan to move to the next goal
    #     self.plan_of_wrist = self.group.plan(self.joint_goal)

    #     # Send waypoints of the goal
    #     self.send_goal()
        
    #     # Execute moving to the joint until movement is reached
    #     self.group.go(self.joint_goal, wait=True)
    #     # Stop movement to ensure that there is no residual movement
    #     self.group.stop()

    def send_goal(self):
        # Get information of the path
        self.amount_of_waypoints = len(self.plan_of_wrist.joint_trajectory.points)
        print (self.amount_of_waypoints)
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
            self.servo_waypoints.theta0[nb_of_waypoints] = int((self.moveit_waypoint_theta0 / pi) * 3140)
            self.servo_waypoints.theta1[nb_of_waypoints] = int((self.moveit_waypoint_theta1 / pi) * 3140)
            self.servo_waypoints.theta2[nb_of_waypoints] = int((self.moveit_waypoint_theta2 / pi) * 3140)
            self.servo_waypoints.theta3[nb_of_waypoints] = int((self.moveit_waypoint_theta3 / pi) * 3140)
        # Print servo_waypoints to check if message was correct
        rospy.loginfo("Waiting ")
        
        while (self.give_waypoints.get_num_connections() == 0):
            pass
     
        self.give_waypoints.publish(self.servo_waypoints)
        rospy.loginfo("message send")
    
            
         


            
    

        
        
        
        

        
        

    # def straight_line(self):
    #     # Storing some hard coded data points
    #     hoekies = codesys_joint()
    #     hoekies.theta0 = [-1.5689095259, -1.5047105551, -1.4402370453, -1.3771352768, -1.3112896681, -1.2422773838, -1.1736456156, -1.1029742956, -1.0318784714, -0.95530653, -0.876789093]
    #     hoekies.theta1 = [0.0037735873, 0.1321716607, 0.2611185312, 0.3873221874, 0.5190133452, 0.6570378542, 0.7943015099, 0.9356440306, 1.0778357983, 1.2309795618, 1.3880144358]
    #     hoekies.theta2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     hoekies.theta3 = [-0.0037735873, -0.1321716607, -0.2611185312, -0.3873221874, -0.5190133452, -0.6570378542, -0.7943015099, -0.9356440306, -1.0778357983, -1.2309795618, -1.3880144358]
        
    #     # Use a for loop to send to through all the data points
    #     for x in range(0,11): 
    #         hoekje = Float32MultiArray()
    #         hoekje.data=[0.0,0.0,0.0,0.0]
    #         hoekje.data[0] = hoekies.theta0[x]
    #         hoekje.data[1] = hoekies.theta1[x]
    #         hoekje.data[2] = 0.0
    #         hoekje.data[3] = hoekies.theta3[x]
    #         self.go_to_joint_state(hoekje)
            
            


if __name__ == '__main__':
    try:
        # Start the main class
        GeneratePathWithMoveIt()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass