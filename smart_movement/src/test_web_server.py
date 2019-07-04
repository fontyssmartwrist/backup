#!/usr/bin/env python
import sys
import rospy
from smart_movement.srv import *
from std_msgs.msg import String


class handle_web_speed_status():
    def __init__(self):

        self.flag = False
        rospy.init_node('web_status_speed_server')
        
        # subscribe to the server topic 
        rospy.Subscriber('/robot_status', String, self.callback)
        self.status_speed_pub = rospy.Publisher('/web_status_speed', String, queue_size=10)
        rospy.loginfo('Waiting for connection')
        rospy.loginfo(self.status_speed_pub.get_num_connections())
        # while (self.status_speed_pub.get_num_connections() == 0 ):
        #     pass
        rospy.loginfo('waiting for the service')
        s = rospy.Service('web_status_speed_srv', RequestStatus, self.web_speed_status)
        
        rospy.spin()
    
    def web_speed_status(self, req):
        # publish the service coming from the web_server to the multithreaded-server
        rospy.loginfo(req.statusreq)
        self.status_speed_pub.publish(req.statusreq)
        rospy.loginfo('Waiting for the response')

        # while self.flag == False:
        #     pass
        self.recieved = 'Motor:[1,1,1,1], Start:[1], Speed:[2], Emergency:[0,0,]'

        
        self.flag = True
        #send the info back to the web server   
        return RequestStatusResponse(self.recieved)
        
    def callback(self, data):
        rospy.loginfo("updated")
        self.flag = True 
        self.recieved = data.data 
        rospy.loginfo(self.recieved)

# def web_status_speed_server():
#     rospy.init_node('web_status_speed_server')
#     # subscribe to the server topic 
#     status_speed_pub = rospy.Publisher('/web_status_speed', String, queue_size=10)
#     while (status_speed_pub.get_num_connections() == 0):
#         rospy.loginfo("waiting for subscriber")
    
#     s = rospy.Service('web_status_speed', RequestStatus, handle_web_speed_status)
#     rospy.spin()











if __name__ == '__main__':
    try:
        handle_web_speed_status()
    
    except rospy.ROSInitException:
        rospy.loginfo("Terminated")
        pass