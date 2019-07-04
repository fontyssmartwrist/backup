#!/usr/bin/env python

import socket
import rospy
import sys
import random
from math import pi

class ImitateCodesysValues():
    
    def __init__(self):

        HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

        # Creat a node
        rospy.init_node('echo_client', anonymous=True)
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        # Prevent node from shutting down
        self.send_to_ros()

    def send_to_ros(self):    
        self.sock.connect(self.server_address)
        
        try:
            while not rospy.is_shutdown():
                full_message = '['
                theta0 = str(int((random.uniform(-pi,pi) / pi) * 100000))
                theta1 = str(int((random.uniform(-pi,pi) / pi) * 100000))
                theta2 = str(int((random.uniform(-pi,pi) / pi) * 100000))
                theta3 = str(int((random.uniform(-pi,pi) / pi) * 100000))
                full_message += theta0 + ',' + theta1 + ',' + theta2 + ',' + theta3 + ']'
                rospy.loginfo(full_message)
                self.sock.sendall(full_message)
                rospy.loginfo("message send")
                rospy.sleep(4)


        finally:
            # Close the socket
            rospy.loginfo('closing socket')
            self.sock.close()


if __name__ == '__main__':
    try:
        # Start the main class
        ImitateCodesysValues()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass    