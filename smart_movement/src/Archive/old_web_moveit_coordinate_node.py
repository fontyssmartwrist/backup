#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg 
import random
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint, CurrentJointValues, smart_joint
from std_msgs.msg import String
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
        self.goal_listen = rospy.Subscriber('/goal_smart_wrist', geometry_msgs.msg.Pose, self.callback)
        # Publish the codesys waypoints to comunication node
        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        # Get the real joint values from codesys
        self.real_listen = rospy.Subscriber('/current_joint_values', String, self.set_current_pos)
        # Send current pose to demo
        self.give_current_pose = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        
        # Move in a straight line demo
        self.straight_line()
            
        # Keep the script running
        rospy.spin()

    def callback(self, goal_data):
        # Store incoming data into angels of goal
        self.data_of_goal = goal_data
        # Make create a path to the goal
        self.go_to_pose_goal(self.data_of_goal)

    def set_current_pos(self, res_msg):
        print(res_msg.data)
        temp = res_msg.data
        first = temp.find('[')
        comma1 = temp.find(',')
        comma2 = temp.find(',',comma1+1)
        comma3 = temp.find(',',comma2+1)
        end = temp.find(']')
        """# Build the current position message
        self.current_pos = JointState()
        self.current_pos.header.seq = 99999
        self.current_pos.header.stamp = rospy.Time.now()
        self.current_pos.header.frame_id = "/world"
        self.current_pos.name = ["end_link", "joint0", "joint1", "joint3", "joint4"]
        self.current_pos.position = [0.0, 
                                    int(temp[first+1:comma1]) * pi / 3140,
                                    int(temp[comma1+1:comma2]) * pi / 3140,
                                    int(temp[comma2+1:comma3]) * pi / 3140,
                                    int(temp[comma3+1:end]) * pi / 3140]
        print self.current_pos
        # Publish the current position to the simulation
        """
        angles = codesys_joint()
        angles.theta0 = int(temp[first+1:comma1]) * pi / 3140
        angles.theta1 = int(temp[comma1+1:comma2]) * pi / 3140
        angles.theta2 = int(temp[comma2+1:comma3]) * pi / 3140
        angles.theta3 = int(temp[comma3+1:end]) * pi / 3140
        self.go_to_joint_state(angles)


    def go_to_pose_goal(self, pose_goal) :
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


    def go_to_joint_state(self, theta):      
        # Set joint goal message type
        self.joint_goal = self.group.get_current_joint_values()
        # Set each joint to the correct value
        self.joint_goal[0] = theta.theta0
        self.joint_goal[1] = theta.theta1
        self.joint_goal[2] = theta.theta2
        self.joint_goal[3] = theta.theta3
        self.joint_goal[4] = 0.0
        # Make a plan to move to the next goal
        self.plan_of_wrist = self.group.plan(self.joint_goal)

        # Send waypoints of the goal
        self.send_goal()
        
        # Execute moving to the joint until movement is reached
        self.group.go(self.joint_goal, wait=True)
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
            self.servo_waypoints.theta0[nb_of_waypoints] = int((self.moveit_waypoint_theta0 / pi) * 3140)
            self.servo_waypoints.theta1[nb_of_waypoints] = int((self.moveit_waypoint_theta1 / pi) * 3140)
            self.servo_waypoints.theta2[nb_of_waypoints] = int((self.moveit_waypoint_theta2 / pi) * 3140)
            self.servo_waypoints.theta3[nb_of_waypoints] = int((self.moveit_waypoint_theta3 / pi) * 3140)
        # Print servo_waypoints to check if message was correct
        print self.servo_waypoints
        # Publish list of waypoints to /codesys_waypoints
        self.give_waypoints.publish(self.servo_waypoints)

    def straight_line(self):
        hoekies = codesys_joint()
        hoekies.theta0 = [-1.5689095259, -1.5047105551, -1.4402370453, -1.3771352768, -1.3112896681, -1.2422773838, -1.1736456156, -1.1029742956, -1.0318784714, -0.95530653, -0.876789093]
        hoekies.theta1 = [0.0037735873, 0.1321716607, 0.2611185312, 0.3873221874, 0.5190133452, 0.6570378542, 0.7943015099, 0.9356440306, 1.0778357983, 1.2309795618, 1.3880144358]
        hoekies.theta2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        hoekies.theta3 = [-0.0037735873, -0.1321716607, -0.2611185312, -0.3873221874, -0.5190133452, -0.6570378542, -0.7943015099, -0.9356440306, -1.0778357983, -1.2309795618, -1.3880144358]
        

        for x in range(10,11): 
            hoekje = smart_joint()
            hoekje.theta0 = hoekies.theta0[x]
            hoekje.theta1 = hoekies.theta1[x]
            hoekje.theta2 = 0.0
            hoekje.theta3 = hoekies.theta3[x]
            hoekje.theta4 = 0.0
            self.go_to_joint_state(hoekje)
            


if __name__ == '__main__':
    try:
        # Start the main class
        GeneratePathWithMoveIt()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass