#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import random
from math import pi
from std_msgs.msg import String

class GiveOrderToMoveIT():
    
    def __init__(self):
        rospy.init_node('sending_goal')
        self.pub = rospy.Publisher('/goal_smart_wrist', geometry_msgs.msg.Pose, queue_size=10)
        sub = rospy.Subscriber('/goal_reached', String, self.callback)
        self.next_goal = False
        self.send_order()


    def send_order(self):
        list_of_ox = [0,       0.28391,     -0.054256,  -0.15593,   0]
        list_of_oy = [0,       -0.31556,    0.39276,    0.11013,    0]
        list_of_oz = [0,       0.88234,     0.047149,   0.96238,    0]
        ist_of_ow =  [1,       0.2032,      0.91683,    0.19337,    1]
        list_of_px = [0,       0.090597,    0.22532,    -0.064657,  0]
        list_of_py = [0,       -0.22011,    0.11558,    0.19571,    0.162325]
        list_of_pz = [0.7453,  0.66702,     0.65988,    0.69223,    0.701805]

        loop = 0 

        while not rospy.is_shutdown():
            if self.next_goal is True:
                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.position.x = list_of_px[loop]
                pose_goal.position.y = list_of_py[loop]
                pose_goal.position.z = list_of_pz[loop]
                pose_goal.orientation.x = list_of_ox[loop]
                pose_goal.orientation.y = list_of_oy[loop]
                pose_goal.orientation.z = list_of_oz[loop]
                pose_goal.orientation.w = list_of_ow[loop]
                rospy.loginfo("message send")
                rospy.loginfo(pose_goal)
                self.pub.publish(pose_goal)
                loop = loop+1
                if loop == 4 :
                    loop = 1
                self.next_goal = False
    
    def callback(self, message):
        rospy.loginfo(message.data)
        if message.data == "Done":
            rospy.loginfo("next_goal = True")
            self.next_goal = True

if __name__ == '__main__':
    try:
        GiveOrderToMoveIT()
    except rospy.ROSInterruptException:
        pass
