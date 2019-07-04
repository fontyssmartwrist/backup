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
        rospy.init_node('client1')
        
        
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        self.server_address = (HOST, PORT)
        rospy.loginfo('connecting to %s port %s', self.server_address[0], self.server_address[1])
        self.send_string = rospy.Publisher('/codesys_data', String, queue_size=100)
        # Prevent node from shutting down
        self.send_to_ros()

    def send_to_ros(self):    
        self.sock.connect(self.server_address)
        self.sock.sendall("Send")
        rospy.sleep(1)
        self.sock.sendall("Done")
        try:
            while not rospy.is_shutdown():
                data = self.sock.recv(1024)
                rospy.loginfo(data)
                if data[0] == 'n':
                    waypoint = str(int(data[4:len(data)-1]) - 1)
                    print(waypoint)
                if data[0] == 'w':
                    self.send_string.publish(data)
                if data[1:3] == waypoint:
                        rospy.sleep(0.5)
                        self.sock.sendall("Done")
                        print("Position reached succesfully")
                if not data:
                    break

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