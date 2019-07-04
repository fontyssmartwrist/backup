#!/usr/bin/env python

import socket
import rospy
import sys
import math
from smart_movement.msg import codesys_joint

class SendWayPointsToCodesys():
    def __init__(self):

        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65430        # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_client', anonymous=True)
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        # Listen to the topic that sends waypoints
        listen = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.send_to_codesys)
        # Prevent node from shutting down
        rospy.spin()
        

    def send_to_codesys(self, data):    
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        self.sock.connect(self.server_address)

        try:
            theta=['','','','']
            # Put message in data
            message = data
            
            for amount in range(0, message.waypoints):
                for joint_nb in range(0,4):
                    theta[joint_nb] += str(message.theta0[amount]) 
                    if amount is message.waypoints -1:
                        theta[joint_nb] += ']'
                    else:
                        theta[joint_nb] += ','  

            # Send max 255 charter long strings
            for joint in range(0,4):
                msg_part = 0
                index = 1
                while not rospy.is_shutdown():
                    msg_end = 245
                    read = theta[joint][msg_part:msg_part + msg_end + 1]
                    msg = 't'+str(joint)+str(index)+':['
                    if not read.rfind(']') is -1:
                        msg_end = read.rfind(']')
                        if read.find(',') is 0:
                            msg = msg + read[1:msg_part + msg_end + 1]
                        else:
                            msg = msg + read[0:msg_part + msg_end + 1]
                    elif index is 1:
                        msg_end = read.rfind(',')
                        msg = msg + read[:msg_part + msg_end] + ']' 
                    else:
                        msg_end = read.rfind(',')
                        msg = msg + read[1:msg_part + msg_end] + ']' 
                    rospy.loginfo(msg)
                    index += 1
                    msg_part = msg_end
                    self.sock.sendall(msg)
                    if not read.rfind(']') is -1:
                        break
        
        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()


       

if __name__ == '__main__':
    try:
        # Start the main class
        SendWayPointsToCodesys()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    