#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray



class ForwardKinematics():
    def __init__(self):
        # Create a node 
        rospy.init_node('forward_kinematics')
        # Get joint goal from the web
        self.web_goal = rospy.Subscriber('/calc_goal', Float32MultiArray, self.callback)
        # Send current pose to demo
        self.give_current_pose = rospy.Publisher('/calculated_end_effector', Pose, queue_size=10)
        #self.get_matrix(theta)
        #print('euler')
        #print(self.rot_2_eul())
        #print('quaternions')
        #print(self.rot_2_quat())
        rospy.spin()
    
    def callback(self, inc_msg):
        theta = [0.0, 0.0, 0.0, 0.0, 0.0]
        theta[0:4] = inc_msg.data[0:4]
        end_effector = Pose()
        xyz = self.get_matrix(theta)
        end_effector.position.x = xyz[0]
        end_effector.position.y = xyz[1]
        end_effector.position.z = xyz[2]
        wxyz = self.rot_2_quat()
        end_effector.orientation.w = wxyz [0]
        end_effector.orientation.x = wxyz [1]
        end_effector.orientation.y = wxyz [2]
        end_effector.orientation.z = wxyz [3]
        self.give_current_pose.publish(end_effector)

    def get_matrix(self, t):
        alfa = (15*math.pi)/180
        d1 = 0.128
        d2 = 0.8

        self.Trans01 = np.matrix([[np.cos(t[0]), -np.sin(t[0]), 0, 0],
                                  [np.sin(t[0]), np.cos(t[0]), 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

        self.Dz12 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])
        self.Ry12 = np.matrix([[np.cos(-alfa), 0, -np.sin(-alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(-alfa), 0, np.cos(-alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Rz12 = np.matrix([[np.cos(t[1]), -np.sin(t[1]), 0, 0],
                                [np.sin(t[1]), np.cos(t[1]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])


        self.Trans02 = self.Trans01*self.Dz12*self.Ry12*self.Rz12

        self.Ry23 = np.matrix([[np.cos(alfa), 0, -np.sin(alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(alfa), 0, np.cos(alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Dz23 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])

        
        self.Trans03 = self.Trans02*self.Ry23*self.Dz23

        self.Dz34 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d2],
                                [0, 0, 0, 1]])

        self.Rz34 = np.matrix([[np.cos(t[2]), -np.sin(t[2]), 0, 0],
                                [np.sin(t[2]), np.cos(t[2]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        
        self.Trans04 = self.Trans03*self.Dz34*self.Rz34

        self.Dz45 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])
        self.Ry45 = np.matrix([[np.cos(-alfa), 0, -np.sin(-alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(-alfa), 0, np.cos(-alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Rz45 = np.matrix([[np.cos(t[3]), -np.sin(t[3]), 0, 0],
                                [np.sin(t[3]), np.cos(t[3]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

        self.Trans05 = self.Trans04*self.Dz45*self.Ry45*self.Rz45

        self.Ry56= np.matrix([[np.cos(alfa), 0, -np.sin(alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(alfa), 0, np.cos(alfa), 0],
                                [0, 0, 0, 1]])        

        self.Dz56 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])

        self.Rz56 = np.matrix([[np.cos(t[4]), -np.sin(t[4]), 0, 0],
                                [np.sin(t[4]), np.cos(t[4]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
      
        self.Trans06 = self.Trans05*self.Ry56*self.Dz56*self.Rz56
        temp_array = np.array([self.Trans06[0,3],self.Trans06[1,3],self.Trans06[2,3]])
        print(temp_array)
        return temp_array

    def six_TF(self, t):
        alfa = (15*math.pi)/180
        d1 = 0.128
        d2 = 0.8

        self.Trans01 = np.matrix([[np.cos(t[0]), -np.sin(t[0]), 0, 0],
                                  [np.sin(t[0]), np.cos(t[0]), 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

        self.Dz12 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])
        self.Ry12 = np.matrix([[np.cos(-alfa), 0, -np.sin(-alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(-alfa), 0, np.cos(-alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Rz12 = np.matrix([[np.cos(t[1]), -np.sin(t[1]), 0, 0],
                                [np.sin(t[1]), np.cos(t[1]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])


        self.Trans02 = np.matrix([[np.cos(-alfa)*np.cos(t[1]), [np.cos(-alfa)*-np.sin(t[1]), -np.sin(-alfa), 0],
                                [np.sin(t[1]), np.cos(t[1]), 0, 0],
                                [np.sin(-alfa)*np.cos(t[1]), np.sin(-alfa)*-np.sin(t[1]), np.cos(-alfa, d1],
                                [0, 0, 0, 1]])

        self.Ry23 = np.matrix([[np.cos(alfa), 0, -np.sin(alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(alfa), 0, np.cos(alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Dz23 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])

        
        self.Trans03 = self.Trans02*self.Ry23*self.Dz23

        self.Dz34 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d2],
                                [0, 0, 0, 1]])

        self.Rz34 = np.matrix([[np.cos(t[2]), -np.sin(t[2]), 0, 0],
                                [np.sin(t[2]), np.cos(t[2]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        
        self.Trans04 = self.Trans03*self.Dz34*self.Rz34

        self.Dz45 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])
        self.Ry45 = np.matrix([[np.cos(-alfa), 0, -np.sin(-alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(-alfa), 0, np.cos(-alfa), 0],
                                [0, 0, 0, 1]])        
        
        self.Rz45 = np.matrix([[np.cos(t[3]), -np.sin(t[3]), 0, 0],
                                [np.sin(t[3]), np.cos(t[3]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

        self.Trans05 = self.Trans04*self.Dz45*self.Ry45*self.Rz45

        self.Ry56= np.matrix([[np.cos(alfa), 0, -np.sin(alfa), 0],
                                [0, 1, 0, 0],
                                [np.sin(alfa), 0, np.cos(alfa), 0],
                                [0, 0, 0, 1]])        

        self.Dz56 = np.matrix([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d1],
                                [0, 0, 0, 1]])

        self.Rz56 = np.matrix([[np.cos(t[4]), -np.sin(t[4]), 0, 0],
                                [np.sin(t[4]), np.cos(t[4]), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
      
        self.Trans06 = self.Trans05*self.Ry56*self.Dz56*self.Rz56
        temp_array = np.array([self.Trans06[0,3],self.Trans06[1,3],self.Trans06[2,3]])
        print(temp_array)
        return temp_array


    def rot_2_eul(self):

        self.sy = math.sqrt(self.Trans06[0,0] * self.Trans06[0,0] +  self.Trans06[1,0] * self.Trans06[1,0])
     
        self.singular = self.sy < 1e-6
 
        if  not self.singular :
            self.x = math.atan2(self.Trans06[2,1] , self.Trans06[2,2])
            self.y = math.atan2(-self.Trans06[2,0], self.sy)
            self.z = math.atan2(self.Trans06[1,0], self.Trans06[0,0])
        else :
            self.x = math.atan2(-self.Trans06[1,2], self.Trans06[1,1])
            self.y = math.atan2(-self.Trans06[2,0], self.sy)
            self.z = 0
 
        return np.array([self.z, self.y, self.x])

    def rot_2_quat(self):

        self.Rot06 = self.Trans06[0:3,0:3]
        
        self.qw = self.Rot06[0,0]+self.Rot06[1,1]+self.Rot06[2,2]
        if self.qw > 0:
            self.q1 = 0.5*math.sqrt(1+self.qw)
        else:
            self.den = np.square(self.Rot06[2,1]-self.Rot06[1,2])+np.square(self.Rot06[0,2]-self.Rot06[2,0])+np.square(self.Rot06[1,0]+self.Rot06[0,1])
            self.num = 3-self.Rot06[0,0]-self.Rot06[1,1]-self.Rot06[2,2]
            self.q1 = 0.5*math.sqrt(self.den/self.num)

        self.qx = self.Rot06[0,0]-self.Rot06[1,1]-self.Rot06[2,2]
        if self.qx > 0:
            self.q2 = 0.5*math.sqrt(1+self.qx)
        else:
            self.den = np.square(self.Rot06[2,1]-self.Rot06[1,2])+np.square(self.Rot06[0,1]+self.Rot06[1,0])+np.square(self.Rot06[2,0]+self.Rot06[0,2])
            self.num = 3-self.Rot06[0,0]+self.Rot06[1,1]+self.Rot06[2,2]
            self.q2 = 0.5*math.sqrt(self.den/self.num)
        
        if (self.Rot06[2,1]-self.Rot06[1,2])<0:
            self.q2 = -self.q2

        self.qy = -self.Rot06[0,0]+self.Rot06[1,1]-self.Rot06[2,2]
        if self.qy > 0:
            self.q3 = 0.5*math.sqrt(1+self.qy)
        else:
            self.den = np.square(self.Rot06[0,2]-self.Rot06[2,0])+np.square(self.Rot06[0,1]+self.Rot06[1,0])+np.square(self.Rot06[1,2]+self.Rot06[2,1])
            self.num = 3+self.Rot06[0,0]-self.Rot06[1,1]+self.Rot06[2,2]
            self.q3 = 0.5*math.sqrt(self.den/self.num)

        if (self.Rot06[0,2]-self.Rot06[2,0])<0:
            self.q3 = -self.q3

        self.qz = -self.Rot06[0,0]-self.Rot06[1,1]+self.Rot06[2,2]
        if self.qz > 0:
            self.q4 = 0.5*math.sqrt(1+self.qz)
        else:
            self.den = np.square(self.Rot06[1,0]-self.Rot06[0,1])+np.square(self.Rot06[2,0]+self.Rot06[0,2])+np.square(self.Rot06[2,1]+self.Rot06[1,2])
            self.num = 3+self.Rot06[0,0]+self.Rot06[1,1]-self.Rot06[2,2]
            self.q4 = 0.5*math.sqrt(self.den/self.num)

        if (self.Rot06[1,0]-self.Rot06[0,1])<0:
            self.q4 = -self.q4

        return np.array([self.q1, self.q2, self.q3, self.q4])  


if __name__ == '__main__':
    try:
        # Start the main class
        ForwardKinematics()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass