#!/usr/bin/env python

import socket
import rospy
import sys
import random
from math import pi
from std_msgs.msg import String

class ImitateCodesysValues():
    
    def __init__(self):

        HOST = '169.254.141.118'  # Standard loopback interface address (localhost)
        PORT = 65432       # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('client2')
        
        
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        self.get_string = rospy.Subscriber('/codesys_data', String, self.codesys)
        self.send_to_ros()

    def send_to_ros(self):    
        self.sock.connect(self.server_address)
        self.sock.sendall("Read")
        try:
            while not rospy.is_shutdown():
                rospy.spin()

        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()

    def codesys(self, data):
        temp = str(data.data)
        colon = temp.find(':')
        new_message = "enc" + temp[colon:]
        rospy.loginfo(new_message)
        self.sock.sendall(new_message)


if __name__ == '__main__':
    try:
        # Start the main class
        ImitateCodesysValues()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    