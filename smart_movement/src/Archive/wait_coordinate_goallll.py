#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import random
from math import pi
from std_msgs.msg import String

class GiveOrderToMoveIT():
    
    def __init__(self):
        rospy.init_node('sending_goal', anonymous=True)
        self.pub = rospy.Publisher('/goal_smart_wrist', geometry_msgs.msg.Pose, queue_size=10)
        sub = rospy.Subscriber('/goal_reached', String, self.callback)
        self.next_goal = True
        self.send_order()


    def send_order(self):
        list_of_ox = [0,       -0.00440,   0,       -0.1129]
        list_of_oy = [0,       -0.00440,   0,       -0.20996]
        list_of_oz = [-pi/2,   -0.033557,  0,       0.36563]
        list_of_ow = [1,       0.99942,    1,       0.89971]
        list_of_px = [0,       0.27364,    0.53069, -0.46675]
        list_of_py = [0,       0.26649,    0,       -0.15256]
        list_of_pz = [1.3229,  1.2517,     1.1806,  1.2139]

        loop = 2 

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
                connections = self.pub.get_num_connections()
                if connections > 0 : 
                    self.pub.publish(pose_goal)
                    rospy.loginfo("message send")
                    rospy.loginfo(pose_goal)
                    loop = loop + 1 
                    if loop == 4 :
                        loop = 1
                    self.next_goal = False
    
    def callback(self, message):
        rospy.loginfo(message.data)
        if message.data == "update":
            rospy.loginfo("next_goal = True")
            self.next_goal = True
       


if __name__ == '__main__':
    try:
        GiveOrderToMoveIT()
    except rospy.ROSInterruptException:
        pass
